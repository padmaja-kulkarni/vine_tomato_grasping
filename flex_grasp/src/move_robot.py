#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

# packages
import sys
import rospy
import moveit_commander

# functions
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list

# services
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# messages
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes

# custom functions
from func.utils import all_close, add_lists
from func.conversions import pose_to_lists

class MoveRobot(object):
    """MoveRobot"""
    def __init__(self):
        super(MoveRobot, self).__init__()

        rospy.init_node("MoveRobot",
                        anonymous=True,
                        log_level=rospy.DEBUG)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass

        self.initialise_robot()
        self.initialise_enviroment()

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.state = "idle"
        self.prev_state = None
        self.command = None

        self.pre_grasp_pose = None
        self.pre_grasp_ee = None
        self.grasp_pose = None
        self.grasp_ee = None
        self.pre_place_pose = None
        self.place_pose = None

        # tolerance
        self.position_tolerance = 0.02 # [m]
        self.orientation_tolerance = 0.02 # [rad]

        # Subscribers
        rospy.Subscriber("preGraspPose", PoseStamped, self.pre_grasp_pose_cb)
        rospy.Subscriber("graspPose", PoseStamped, self.grasp_pose_cb)
        rospy.Subscriber("prePlacePose", PoseStamped, self.pre_place_pose_cb)
        rospy.Subscriber("placePose", PoseStamped, self.place_pose_cb)
        # rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers
        latch = False
        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=latch)

    # def ee_distance_cb(self, msg):
    #     dist = msg.data
    #     if self.EE_GRASP is None:
    #         dist = msg.data
    #         self.EE_GRASP = add_lists(self.EE_CLOSED, [dist/2, -dist/2])
    #         rospy.logdebug("[MOVE ROBOT] Received a new end effector distance message")
    #         rospy.logdebug("[MOVE ROBOT] New end effector grasp pose: %s", self.EE_GRASP)

    def grasp_pose_cb(self, msg):
        if self.grasp_pose is None:
            self.grasp_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new grasp pose massage")

            if rospy.has_param('grasp_ee'):
                self.grasp_ee = rospy.get_param('grasp_ee')
            else:
                rospy.logwarn("[MOVE ROBOT] Grasp end effector pose can not be loaded from parameter server")


    def pre_grasp_pose_cb(self, msg):
        if self.pre_grasp_pose is None:
            self.pre_grasp_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new pre grasp pose massage")

            if rospy.has_param('pre_grasp_ee'):
                self.pre_grasp_ee = rospy.get_param('pre_grasp_ee')
            else:
                rospy.logwarn("[MOVE ROBOT] Pre grasp end effector pose can not be loaded from parameter server")

    def pre_place_pose_cb(self, msg):
        if self.pre_place_pose is None:
            self.pre_place_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new pre place pose message")

    def place_pose_cb(self, msg):
        if self.place_pose is None:
            self.place_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new placing pose message")

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[MOVE ROBOT] Received new move robot event in message: %s", self.command)


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

        rospy.logdebug("[MOVE ROBOT] Manipulator planning frame: %s", man_planning_frame)
        rospy.logdebug("[MOVE ROBOT] End effector planning frame: %s", ee_planning_frame)

        ee_link = man_group.get_end_effector_link()

        manipulator_joint_names = man_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys() # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()

        # Allow replanning to increase the odds of a solution
        man_group.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        man_group.set_planning_time(5)

        # Allow some leeway in position (meters) and orientation (radians)
        # man_group.set_goal_position_tolerance(0.05)
        # man_group.set_goal_orientation_tolerance(0.1)

        # ee_group.set_goal_position_tolerance(0.05)
        # ee_group.set_goal_orientation_tolerance(0.1)

        self.max_pick_attempts = 10
        self.max_place_attempts = 10

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
        while True:
            known_objects = self.scene.get_known_object_names()

            if not known_objects == known_objects_prev:
                known_objects_prev = known_objects
                rospy.logdebug("[MOVE ROBOT] Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("[MOVE ROBOT] Table and wall object not present...")
                    break
            rospy.sleep(0.1)

        rospy.logdebug("[MOVE ROBOT] Known objects: %s", self.scene.get_known_object_names())


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
        # remove initial "/"
        planning_frame = self.man_planning_frame[1:]

        if goal_frame == planning_frame:
            return True
        else:
            rospy.logwarn("[MOVE ROBOT] Goal pose specified with respect to wronf frame: sould be specified with respect to %s, but is be specified with respect to %s", planning_frame, goal_frame)
            return False


    def pick(self):
        rospy.logdebug("[MOVE ROBOT] Picking object")
        success =  self.go_to_pre_grasp_pose()

        if success:
            success = self.apply_pre_grasp_ee()

        if success:
            success = self.go_to_grasp_pose()

        if success:
            success = self.apply_grasp_ee()

        if success:
            success = self.go_to_pre_grasp_pose()

        return success

    def place(self):
        rospy.logdebug("[MOVE ROBOT] Placing object")
        success = self.go_to_pre_place_pose()

        if success:
            success = self.go_to_place_pose()

        if success:
            success = self.apply_release_ee()

        if success:
            success = self.go_to_pre_place_pose()

        if success:
            success = self.home_man()


        self.reset_msg()
        return success

    def remove_attached_target_object(self):
        attached_objects = self.scene.get_attached_objects(object_ids=[self.target_object_name])

        if self.target_object_name in attached_objects.keys():
            if attached_objects[self.target_object_name].link_name == self.ee_link:
                rospy.logdebug("[MOVE ROBOT] Removing atached objects from ee_link")
                self.scene.remove_attached_object(self.ee_link, self.target_object_name)
                rospy.sleep(0.1)
                self.scene.remove_world_object(self.target_object_name)
                return True
            else:
                rospy.logwarn("[MOVE ROBOT] Cannot remove attached object: target object is not attached to the end effector link!")
                return False
        else:
            rospy.logwarn("[MOVE ROBOT] Cannot remove attached object: target object is not attached to anything!")
            return False

    def attach_object(self):
        grasping_group = self.ee_group_name
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.ee_link, self.target_object_name, touch_links=touch_links)
        rospy.sleep(0.1)
        return True

    def go_to_random_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to random pose")
        goal_pose = self.man_group.get_random_pose()
        success = self.go_to_pose(goal_pose)
        # self.grasp_pose = None
        return success

    def go_to_grasp_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to grasp pose")
        success = self.go_to_pose(self.grasp_pose)
        # self.grasp_pose = None
        return success

    def go_to_pre_grasp_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to pre grasp pose")
        success = self.go_to_pose(self.pre_grasp_pose)
        # self.pre_grasp_pose = None
        return success

    def go_to_pre_place_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to pre place pose")
        success = self.go_to_pose(self.pre_place_pose)
        return success

    def go_to_place_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to place pose")
        success = self.go_to_pose(self.place_pose)
        return success

    def go_to_pose(self, goal_pose):

        if not self.check_frames(goal_pose.header.frame_id):
            return False
        ## Only if inverse kinematics exist
        if not self.check_ik(goal_pose):
            return False

        self.man_group.set_pose_target(goal_pose.pose)
        plan = self.man_group.plan()
        self.man_group.execute(plan, wait=True)

        # Ensures that there is no residual movement and clear the target
        self.man_group.stop()
        self.man_group.clear_pose_targets()

        curr_pose = self.man_group.get_current_pose()
        success = all_close(goal_pose, curr_pose, self.position_tolerance, self.orientation_tolerance)

        if success is False:
            rospy.logwarn("[MOVE ROBOT] Failed to move to pose target, obtained pose is not sufficiently close to goal pose!")
            rospy.loginfo("[MOVE ROBOT] Goal pose: %s", pose_to_list(goal_pose.pose))
            rospy.loginfo("[MOVE ROBOT] Curr pose: %s", pose_to_list(curr_pose.pose))
        return success

    def move_to_joint_target(self, group, target):

        # if the target is a named target, get the corresponding joint values
        if type(target) is str:
            target = group.get_named_target_values(target)

        group.set_joint_value_target(target)

        plan = group.plan()
        group.execute(plan, wait=True)
        group.stop()

        # target = group.get_joint_value_target() #get_remembered_joint_values().values()
        actual = group.get_current_joint_values()
        success = all_close(target, actual, self.position_tolerance, self.orientation_tolerance)

        if success is False:
            rospy.logwarn("[MOVE ROBOT] Failed to move to joint target: obtained joint values are not sufficiently close to goal pose!")
            rospy.loginfo("[MOVE ROBOT] Target joint values: %s", target)
            rospy.loginfo("[MOVE ROBOT] Actual joint values: %s", actual)

        group.clear_pose_targets()
        # rospy.logdebug("[MOVE ROBOT] Moving to joint target success: %s", success)
        return success

    def open_ee(self):
        rospy.logdebug("[MOVE ROBOT] Opening end effector")
        return self.move_to_joint_target(self.ee_group, "Open")

    def close_ee(self):
        rospy.logdebug("[MOVE ROBOT] Closing end effector")
        return self.move_to_joint_target(self.ee_group, "Closed")

    def home_man(self):
        rospy.logdebug("[MOVE ROBOT] Homeing manipulator")
        return self.move_to_joint_target(self.man_group, 'Upright')

    def apply_release_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying release with end effector")

        success = self.open_ee()
        if success:
            success = self.remove_attached_target_object()
        return success

    def apply_grasp_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying grasping with end effector")

        success = self.attach_object()
        if success:
            success = self.move_to_joint_target(self.ee_group, self.grasp_ee)
        return success

    def apply_pre_grasp_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying pre grasping with end effector")

        return self.move_to_joint_target(self.ee_group, self.pre_grasp_ee)

    def received_all_data(self):
        success = (self.pre_grasp_ee != None) and (self.grasp_ee != None) and (self.grasp_pose != None) and (self.pre_grasp_pose != None) and (self.pre_place_pose != None) and (self.place_pose != None)
        return success

    def reset_msg(self):
        rospy.logdebug("[MOVE ROBOT] Resetting for next grasp")
        self.pre_grasp_ee = None
        self.grasp_ee = None
        self.grasp_pose = None
        self.pre_grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None

    ### Log state update
    def log_state_update(self):
        rospy.loginfo("[MOVE ROBOT] updated move robot state, from %s to %s",
                      self.prev_state, self.state)

    def update_state(self, success):

        if (self.state == "idle") and not self.received_all_data():
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()

        elif self.state == "init" and self.received_all_data():
            self.prev_state = self.state
            self.state = "idle"
            self.log_state_update()

        elif (self.state == "idle") and (self.command == "pick") and success:
            self.prev_state = self.state
            self.state = "picked"
            self.log_state_update()

        elif (self.state == "picked") and success: # and (self.command == "place")
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()


    def take_action(self):
        msg = String()
        success = None

        # State dependent actions
        if self.state == "idle":
            if self.command == "pick":
                success = self.pick()

        elif self.state == "picked":
            # if self.command == "place":
            success = self.place()

        # General actions, non state dependent
        if self.command == "move":
            success = self.go_to_pre_grasp_pose()

        elif self.command == "home":
            success = self.home_man()

        elif self.command == "open":
            success = self.open_ee()

        elif self.command == "close":
            success = self.close_ee()

        elif self.command == "e_init":
            success = True

        self.update_state(success)

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.command = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Robot command failed")
                self.command = None

            self.pub_e_out.publish(msg)

def main():
    try:
        move_robot = MoveRobot()

        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            move_robot.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
