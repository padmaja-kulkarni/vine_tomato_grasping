#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

# packages
import numpy as np
import sys
import rospy
import moveit_commander
from flex_shared_resources.utils.communication import Communication
import tf2_ros
import tf2_geometry_msgs

# functions
from moveit_commander.conversions import pose_to_list

# services
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from flex_grasp.msg import FlexGraspErrorCodes

from interbotix_sdk.msg import SingleCommand
from interbotix_sdk.srv import OperatingModes, OperatingModesRequest
from interbotix_sdk.srv import RobotInfo
from sensor_msgs.msg import JointState

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log

# custom functions
from func.utils import pose_close, joint_close


class MoveRobot(object):
    """MoveRobot"""
    node_name = "MOVE_ROBOT"

    def __init__(self):
        super(MoveRobot, self).__init__()

        self.debug_mode = rospy.get_param("move_robot/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[MOVE ROBOT] Launching move robot in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("move_robot", anonymous=True, log_level=log_level)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass

        self.force_robot = False
        self.state = "idle"
        self.prev_state = None
        self.command = None
        self.robot_pose = None
        self.rate = rospy.Rate(10)
        self.simulation = self.debug_mode = rospy.get_param("robot_sim")

        # tolerance
        self.position_tol = 0.03  # [m]
        self.orientation_tol = np.deg2rad(10.0)  # [rad]
        self.man_joint_tolerance = np.deg2rad(5.0)  # [rad]
        self.ee_joint_tolerance = 0.002  # [m]

        self.initialise_robot()
        self.initialise_enviroment()

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        # Subscribers
        rospy.Subscriber("robot_pose", PoseStamped, self.robot_pose_cb)

        # rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers
        self.pub_e_out = rospy.Publisher("~e_out",
                                         FlexGraspErrorCodes, queue_size=10, latch=True)

        # init monitor robot communication
        monitor_robot_topic = "monitor_robot"
        self.monitor_robot_communication = Communication(monitor_robot_topic, timeout=15)

    def robot_pose_cb(self, msg):
        if self.robot_pose is None:
            if msg.header.frame_id != self.man_planning_frame:
                msg = self.transform_pose(msg, self.man_planning_frame)

            self.robot_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new robot pose massage")

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[MOVE ROBOT] Received new move robot event in message: %s", self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)

    ### @brief ROS Subscriber Callback function to update the latest arm joint states
    ### @param msg - latest JointState message
    ### @details - the JointState message is mainly used to determine current gripper position
    def joint_state_cb(self, msg):
        self.joint_states = msg

    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        ## This object is an interface to one group of joints.
        man_group_name = rospy.get_param('manipulator_group_name')
        ee_group_name = rospy.get_param('ee_group_name')

        rospy.logdebug("[MOVE ROBOT] Manipulator group name: %s", man_group_name)
        rospy.logdebug("[MOVE ROBOT] End effector group name: %s", ee_group_name)

        man_group = moveit_commander.MoveGroupCommander(man_group_name)
        ee_group = moveit_commander.MoveGroupCommander(ee_group_name)

        man_planning_frame = man_group.get_planning_frame()
        ee_planning_frame = ee_group.get_planning_frame()
        ee_link = man_group.get_end_effector_link()

        rospy.logdebug("[MOVE ROBOT] Manipulator planning frame: %s", man_planning_frame)
        rospy.loginfo("[MOVE ROBOT] Manipulator end effector link: %s", ee_link) # px150/ee_arm_link
        rospy.logdebug("[MOVE ROBOT] End effector planning frame: %s", ee_planning_frame)


        manipulator_joint_names = man_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys()  # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()

        # Allow replanning to increase the odds of a solution
        man_group.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        man_group.set_planning_time(5)

        man_group.set_max_velocity_scaling_factor(0.8)
        man_group.set_max_acceleration_scaling_factor(0.7)

        # Allow some leeway in position (meters) and orientation (radians) affects PLANNING!
        man_group.set_goal_position_tolerance(0.005)
        man_group.set_goal_orientation_tolerance(np.deg2rad(0.5))
        man_group.set_goal_joint_tolerance(np.deg2rad(0.5))
        #
        #        ee_group.set_goal_position_tolerance(self.position_tol)
        #        ee_group.set_goal_orientation_tolerance(self.orientation_tol)
        #        ee_group.set_goal_joint_tolerance(self.ee_joint_tol)

        if not self.simulation:
            rospy.wait_for_service("get_robot_info")
            srv_robot_info = rospy.ServiceProxy("get_robot_info", RobotInfo)
            self.resp = srv_robot_info()
            self.num_joints = self.resp.num_joints
            self.gripper_index = self.num_joints + 1

            self.set_operating_mode_srv = rospy.ServiceProxy('set_operating_modes', OperatingModes)
            self.pub_gripper_command = rospy.Publisher("single_joint/command",
                                                       SingleCommand, queue_size=10, latch=True)

            self.sub_joint_states = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)

            self.gripper_command = SingleCommand()
            self.gripper_command.joint_name = 'gripper'
            self.gripper_command.cmd = 0

            self.joint_states = JointState()
            self.close_pwm_cmd = -250
            self.open_pwm_cmd = 250

        self.man_group_name = man_group_name
        self.ee_group_name = ee_group_name
        self.robot = robot

        self.man_group = man_group
        self.ee_group = ee_group

        self.man_planning_frame = man_planning_frame
        self.ee_planning_frame = ee_planning_frame

        self.ee_link = ee_link
        self.ee_joint_names = ee_joint_names
        self.EE_OPEN = EE_OPEN
        self.EE_CLOSED = EE_CLOSED

    def initialise_enviroment(self):
        """" Checks wether the RViz enviroment is correctly set

        """

        rospy.logdebug("===INITIALIZING ENVIROMENT====")

        # interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        self.target_object_name = 'peduncle'

        known_objects_prev = None
        required_object = 'table'
        found_required_object = False
        start_time = rospy.get_time()
        curr_time = rospy.get_time()
        timeout = 10

        while (curr_time - start_time < timeout) and not found_required_object:
            known_objects = self.scene.get_known_object_names()

            if not known_objects == known_objects_prev:
                known_objects_prev = known_objects
                rospy.logdebug("[MOVE ROBOT] Known objects: %s", known_objects)

                if (required_object in known_objects):
                    rospy.logdebug("[MOVE ROBOT] Continueing initialization, required object is present")
                    found_required_object = True
                else:
                    rospy.logwarn("[MOVE ROBOT] Refusing to continue until %s is present.", required_object)
                    found_required_object = True # False
            self.rate.sleep()

    def check_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Checks inverse kinematics for the given pose.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped

        request.ik_request.group_name = self.man_group_name
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)

        if not response.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.logwarn("[MOVE ROBOT] Goal pose is not reachable: inverse kinematics can not be found")
            return False
        else:
            return True

    def check_frames(self, goal_frame):
        # remove pre appended is present "/"
        if self.man_planning_frame[1:] == '/':
            planning_frame = self.man_planning_frame[1:]
        else:
            planning_frame = self.man_planning_frame

        if goal_frame == planning_frame:
            return True
        else:
            rospy.logwarn("[MOVE ROBOT] Goal pose specified with respect to wrong frame: should be specified with respect to %s, but is specified with respect to %s",
                planning_frame, goal_frame)
            return False

    def remove_attached_target_object(self):
        attached_objects = self.scene.get_attached_objects(object_ids=[self.target_object_name])

        if self.target_object_name in attached_objects.keys():
            if attached_objects[self.target_object_name].link_name == self.ee_link:
                rospy.logdebug("[MOVE ROBOT] Removing attached objects from ee_link")
                self.scene.remove_attached_object(self.ee_link, self.target_object_name)
                self.rate.sleep()
                self.scene.remove_world_object(self.target_object_name)
                return True
            else:
                rospy.logdebug(
                    "[MOVE ROBOT] Cannot remove attached object: target object is not attached to the end effector link!")
                return True
        else:
            rospy.logdebug("[MOVE ROBOT] Cannot remove attached object: target object is not attached to anything!")
            return True

    def attach_object(self):
        grasping_group = self.ee_group_name
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.ee_link, self.target_object_name, touch_links=touch_links)
        self.rate.sleep()
        return FlexGraspErrorCodes.SUCCESS

    def go_to_random_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to random pose")
        goal_pose = self.man_group.get_random_pose()
        success = self.go_to_pose(goal_pose)
        # self.grasp_pose = None
        return success

    def sleep_man(self):
        rospy.logdebug("[MOVE ROBOT] Sleeping manipulator")
        return self.move_to_joint_target(self.man_group, 'Sleep')

    def home_man(self):
        rospy.logdebug("[MOVE ROBOT] Homeing manipulator")
        return self.move_to_joint_target(self.man_group, 'Upright')

    def ready_man(self):
        rospy.logdebug("[MOVE ROBOT] Moving manipulator to ready joint target")
        return self.move_to_joint_target(self.man_group, 'Home')

    def open_ee(self):
        rospy.logdebug("[MOVE ROBOT] Opening end effector")
        if self.simulation:
            return self.move_to_joint_target(self.ee_group, "Open")
        else:
            self.set_operating_mode()
            return self.move_to_joint_target_pwm(self.open_pwm_cmd)

    def close_ee(self):
        rospy.logdebug("[MOVE ROBOT] Closing end effector")
        if self.simulation:
            return self.move_to_joint_target(self.ee_group, "Closed")
        else:
            self.set_operating_mode()
            return self.move_to_joint_target_pwm(self.close_pwm_cmd)

    def apply_release_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying release with end effector")

        result = self.open_ee()
        if result == FlexGraspErrorCodes.SUCCESS:
            self.remove_attached_target_object()
        return result

    def apply_grasp_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying grasping with end effector")

        result = self.attach_object()
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.close_ee()

        return result

    def monitor_group(self, group):
        """Ask permission from robot monitor"""

        group_name = group.get_name()
        if group_name == self.man_group_name:
            command = "monitor_arm"
        elif group_name == self.ee_group_name:
            command = "monitor_gripper"

        return self.monitor_robot_communication.wait_for_result(command)

    def go_to_pose(self, goal_pose, robot_force=False):
        rospy.logdebug("[MOVE ROBOT] Go to pose")

        if goal_pose is None:
            return FlexGraspErrorCodes.NO_GOAL_POSE

        if not self.check_frames(goal_pose.header.frame_id):
            return FlexGraspErrorCodes.INVALID_FRAME

        if not self.check_ik(goal_pose):
            return FlexGraspErrorCodes.NO_IK_SOLUTION

        result = self.monitor_group(self.man_group)
        if (result is not FlexGraspErrorCodes.SUCCESS) and (not self.force_robot):
            return result

        self.man_group.set_pose_target(goal_pose.pose)
        plan = self.man_group.plan()

        # if no plan found... very ugly, what is a better way to do this?
        if len(plan.joint_trajectory.joint_names) == 0:
            rospy.logwarn('[MOVE ROBOT] not attempting to go to pose goal, no plan found!')
            self.man_group.clear_pose_targets()
            return FlexGraspErrorCodes.PLANNING_FAILED

        success = self.man_group.execute(plan, wait=True)
        if success is not True:
            rospy.logwarn('[MOVE ROBOT] Controller failed during execution, trying to continue...')
            # return FlexGraspErrorCodes.CONTROL_FAILED

        # Ensures that there is no residual movement and clear the target
        rospy.logdebug("[MOVE ROBOT] executed plan")
        self.man_group.stop()
        self.man_group.clear_pose_targets()

        result = self.wait_for_pose_close(self.man_group, goal_pose,
                                          self.position_tol,
                                          self.orientation_tol, 3)

        return result

    def set_operating_mode(self):
        request = OperatingModesRequest()
        request.cmd = 3  # gripper
        request.mode = 'pwm'
        request.use_custom_profiles = False
        self.set_operating_mode_srv(request)

    def move_to_joint_target_pwm(self, command, timeout=2.0):
        rospy.logdebug("[MOVE ROBOT] Set PWM target")

        if self.force_robot:
            rospy.loginfo("[MOVE ROBOT] Forcing to execute command!")
        else:
            result = self.monitor_group(self.ee_group)
            if result != FlexGraspErrorCodes.SUCCESS:
                return result

        self.gripper_command.cmd = command
        self.pub_gripper_command.publish(self.gripper_command)

        start_time = rospy.get_time()
        curr_time = rospy.get_time()

        while (curr_time - start_time < timeout) and not rospy.is_shutdown():
            js_msg = list(self.joint_states.position)
            if len(js_msg) != 0:

                if ((self.gripper_command.cmd > 0 and js_msg[self.gripper_index] >= self.EE_OPEN[0]) or
                        (self.gripper_command.cmd < 0 and js_msg[self.gripper_index] <= self.EE_CLOSED[0])):
                    self.gripper_command.cmd = 0
                    self.pub_gripper_command.publish(self.gripper_command)
                    return FlexGraspErrorCodes.SUCCESS

            if rospy.is_shutdown():
                return FlexGraspErrorCodes.SHUTDOWN

            curr_time = rospy.get_time()
            self.rate.sleep()

        if self.gripper_command.cmd > 0:
            js_target = self.EE_OPEN[0]
        elif self.gripper_command.cmd < 0:
            js_target = self.EE_CLOSED[0]
        else:
            js_target = None

        rospy.logwarn("[{0}] Control failed: gripper did not reach target within allocated time".format(self.node_name))
        rospy.loginfo("[{0}] joint state target is {1}".format(self.node_name, js_target))
        rospy.loginfo("[{0}] joint state actual is {1}".format(self.node_name, js_msg[self.gripper_index]))
        return FlexGraspErrorCodes.CONTROL_FAILED

    def move_to_joint_target(self, group, target):
        rospy.logdebug("[{0}] Go to joint target".format(self.node_name))
        result = FlexGraspErrorCodes.SUCCESS

        if self.force_robot:
            rospy.loginfo("[{0}] Forcing to execute command!".format(self.node_name))
        else:
            result = self.monitor_group(group)
            if (result != FlexGraspErrorCodes.SUCCESS):
                return result
        to_check = True

        # if the target is a named target, get the corresponding joint values
        if type(target) is str:
            if target == "Sleep":
                to_check = False
            target = group.get_named_target_values(target)

        group.set_joint_value_target(target)

        plan = group.plan()
        success = group.execute(plan, wait=True)
        if success is not True:
            rospy.logwarn('[MOVE ROBOT] Controller failed during excution!')
            return FlexGraspErrorCodes.CONTROL_FAILED
        group.stop()

        #  group.get_joint_value_target() returns zeros on ee_group_name
        # target.values() returns values in the wrong order on man_group
        if (group.get_name() == self.ee_group_name):
            target_val = target.values()
        else:
            target_val = group.get_joint_value_target()

        if (group.get_name() == self.ee_group_name):
            tol = self.ee_joint_tolerance
        else:
            tol = self.man_joint_tolerance

            # oscilations can cause the robot to be at and different pose than desired, therefore we check several times
        if to_check:
            result = self.wait_for_joint_close(group, target_val, tol, 2.0)

        group.clear_pose_targets()
        return result

    def wait_for_pose_close(self, group, goal_pose, pos_tol, or_tol, timeout):
        start_time = rospy.get_time()
        curr_time = rospy.get_time()

        while curr_time - start_time < timeout:

            current_pose = group.get_current_pose()

            position_close, orientation_close = pose_close(goal_pose, current_pose, pos_tol, or_tol)
            if position_close and orientation_close:
                return FlexGraspErrorCodes.SUCCESS

            self.rate.sleep()
            curr_time = rospy.get_time()

        if orientation_close == False:
            rospy.logwarn(
                "[MOVE ROBOT] Failed to move to pose target, obtained orientation is not sufficiently close to goal orientation (tolerance: %s)",
                or_tol)

        if position_close == False:
            rospy.logwarn(
                "[MOVE ROBOT] Failed to move to pose target, obtained position is not sufficiently close to goal position (tolerance: %s)",
                pos_tol)

        rospy.logdebug("[MOVE ROBOT] Goal pose: %s", pose_to_list(goal_pose.pose))
        rospy.logdebug("[MOVE ROBOT] Curr pose: %s", pose_to_list(current_pose.pose))
        return FlexGraspErrorCodes.CONTROL_FAILED

    def wait_for_joint_close(self, group, target, tol, timeout):
        start_time = rospy.get_time()
        curr_time = rospy.get_time()

        while curr_time - start_time < timeout:

            current = group.get_current_joint_values()

            if len(current) == 0:
                rospy.logwarn("[MOVE ROBOT] Failed to get current robot state")
                group.clear_pose_targets()
                return FlexGraspErrorCodes.CONTROL_FAILED

            if joint_close(target, current, tol):
                return FlexGraspErrorCodes.SUCCESS

            self.rate.sleep()
            curr_time = rospy.get_time()

        rospy.logwarn(
            "[MOVE ROBOT] Failed to move to joint target: obtained joint values are not sufficiently close to target joint value (tolerance: %s)",
            tol)
        rospy.loginfo("[MOVE ROBOT] Target joint values: %s", target)
        rospy.loginfo("[MOVE ROBOT] Actual joint values: %s", current)
        return FlexGraspErrorCodes.CONTROL_FAILED

    def wait_for_robot_pose(self, timeout=1):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time < timeout):
            if self.robot_pose is not None:
                return FlexGraspErrorCodes.SUCCESS

            rospy.sleep(0.1)

        return FlexGraspErrorCodes.NO_GOAL_POSE

    def transform_pose(self, pose_stamped, to_frame):
        original_frame = pose_stamped.header.frame_id

        try:
            transform = self.tfBuffer.lookup_transform(to_frame, original_frame, time=rospy.Time.now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[%s] Cannot transform pose, failed to lookup transform from %s to %s!", self.node_name,
                          original_frame, to_frame)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    def reset(self):
        rospy.logdebug("[MOVE ROBOT] Resetting robot pose")
        self.robot_pose = None
        return FlexGraspErrorCodes.SUCCESS

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        # General actions, non state dependent
        if self.command == "move_manipulator":
            # rospy.loginfo("[MOVE ROBOT] Going to pose...")
            result = self.wait_for_robot_pose()
            if result == FlexGraspErrorCodes.SUCCESS:
                result = self.go_to_pose(self.robot_pose)
            # rospy.loginfo("[MOVE ROBOT]  Result: %s", move_robot_result_string(result))
            self.robot_pose = None

        elif self.command == "sleep":
            result = self.sleep_man()

        elif self.command == "force_robot":
            self.force_robot = True
            result = FlexGraspErrorCodes.SUCCESS

        elif self.command == "do_not_force_robot":
            self.force_robot = False
            result = FlexGraspErrorCodes.SUCCESS

        elif self.command == "home":
            result = self.home_man()

        elif self.command == "ready":
            result = self.ready_man()

        elif self.command == "grasp":
            result = self.apply_grasp_ee()

        elif self.command == "release":
            result = self.apply_release_ee()

        elif self.command == "open":
            result = self.apply_release_ee()

        elif self.command == "close":
            result = self.close_ee()

        elif self.command == "reset":
            result = self.reset()

        elif self.command == "e_init":
            result = FlexGraspErrorCodes.SUCCESS

        elif self.command == None:
            pass

        else:
            rospy.logwarn("Received unknown command: %s", self.command)
            result = FlexGraspErrorCodes.UNKNOWN_COMMAND

        # publish result
        if result is not None:
            flex_grasp_error_log(result, self.node_name)
            msg.val = result
            self.pub_e_out.publish(msg)
            self.command = None


def main():
    try:
        move_robot = MoveRobot()

        while not rospy.core.is_shutdown():
            move_robot.take_action()
            move_robot.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
