#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

import sys
import rospy
import moveit_commander

from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

class Pick_Place(object):
    """Pick_Place"""
    def __init__(self):
        super(Pick_Place, self).__init__()

        rospy.init_node("Pick_Place",
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
        self.goal_state = None

        self.pre_grasp_pose = None
        self.grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None

        self.position_tolerance = 0.02 # [m]
        self.orientation_tolerance = 0.02 # [rad]

        # Subscribers
        rospy.Subscriber("preGraspPose", PoseStamped, self.pre_grasp_pose_cb)
        rospy.Subscriber("graspPose", PoseStamped, self.grasp_pose_cb)
        rospy.Subscriber("prePlacePose", PoseStamped, self.pre_place_pose_cb)
        rospy.Subscriber("placePose", PoseStamped, self.place_pose_cb)
        rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers
        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=True)

    def ee_distance_cb(self, msg):
        dist = msg.data
        if self.EE_GRASP is None:
            dist = msg.data
            self.EE_GRASP = add_lists(self.EE_CLOSED, [dist/2, -dist/2])
            rospy.logdebug("[PICK PLACE] Received a new end effector distance message")
            rospy.logdebug("[PICK PLACE] New end effector grasp pose: %s", self.EE_GRASP)

    def grasp_pose_cb(self, msg):
        if self.grasp_pose is None:
            self.grasp_pose = msg
            rospy.logdebug("[PICK PLACE] Received new grasp pose massage")

    def pre_grasp_pose_cb(self, msg):
        if self.pre_grasp_pose is None:
            self.pre_grasp_pose = msg
            rospy.logdebug("[PICK PLACE] Received new pre grasp pose massage")

    def pre_place_pose_cb(self, msg):
        if self.pre_place_pose is None:
            self.pre_place_pose = msg
            rospy.logdebug("[PICK PLACE] Received new pre place pose message")

    def place_pose_cb(self, msg):
        if self.place_pose is None:
            self.place_pose = msg
            rospy.logdebug("[PICK PLACE] Received new placing pose message")

    def e_in_cb(self, msg):
        if self.goal_state is None:
            self.goal_state = msg.data
            rospy.logdebug("[PICK PLACE] Received new move robot event in message: %s", self.goal_state)


    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        ## This object is an interface to one group of joints.
        man_group_name = rospy.get_param('manipulator_group_name')
        ee_group_name = rospy.get_param('ee_group_name')

        man_group = moveit_commander.MoveGroupCommander(man_group_name)
        ee_group = moveit_commander.MoveGroupCommander(ee_group_name)

        ee_link = man_group.get_end_effector_link()

        manipulator_joint_names = man_group.get_joints()
        # ee_joint_names = ee_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys() # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()
        EE_GRASP = None

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

        self.ee_link = ee_link
        self.ee_joint_names = ee_joint_names
        self.EE_OPEN = EE_OPEN
        self.EE_CLOSED = EE_CLOSED
        self.EE_GRASP = EE_GRASP

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
                rospy.logdebug("Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("[Pick Place] Table and wall object not present...")
                    break
            rospy.sleep(0.1)

        rospy.logdebug( "Known objects: %s", self.scene.get_known_object_names())


    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

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
            return False
        else:
            return True

    def pick(self):
        success =  self.go_to_pre_grasp_pose()

        if success:
            success = self.open_ee()

        if success:
            success = self.go_to_grasp_pose()

        if success:
            success = self.attach_object()

        if success:
            success = self.close_ee()

        if success:
            success = self.go_to_pre_grasp_pose()

        return success

    def place(self):
        success = self.go_to_pre_place_pose()

        if success:
            success = self.go_to_place_pose()

        if success:
            success = self.open_ee()

        if success:
            success = self.go_to_pre_place_pose()

        if success:
            success = self.home_man()

        return success

    def remove_attached_target_object(self):
        attached_objects = self.scene.get_attached_objects(object_ids=[self.target_object_name])

        if self.target_object_name in attached_objects.keys():
            if attached_objects[self.target_object_name].link_name == self.ee_link:
                rospy.logdebug("Removing atached objects from ee_link")
                self.scene.remove_attached_object(self.ee_link, self.target_object_name)
                rospy.sleep(0.1)
                self.scene.remove_world_object(self.target_object_name)

    def attach_object(self):
        grasping_group = self.ee_group_name
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.ee_link, self.target_object_name, touch_links=touch_links)
        rospy.sleep(0.1)
        return True

    def go_to_grasp_pose(self):
        rospy.logdebug("[PICK PLACE] Going to grasp pose")
        success = self.go_to_pose(self.grasp_pose)
        # self.grasp_pose = None
        return success

    def go_to_pre_grasp_pose(self):
        rospy.logdebug("[PICK PLACE] Going to pre grasp pose")
        success = self.go_to_pose(self.pre_grasp_pose)
        # self.pre_grasp_pose = None
        return success

    def go_to_pre_place_pose(self):
        rospy.logdebug("[PICK PLACE] Going to pre place pose")
        success = self.go_to_pose(self.pre_place_pose)
        return success

    def go_to_place_pose(self):
        rospy.logdebug("[PICK PLACE] Going to place pose")
        success = self.go_to_pose(self.place_pose)
        return success

    def go_to_pose(self, goal_pose):
        """ plan and move to a pose goal


        """

        ## Only if inverse kinematics exist
        if self.compute_ik(goal_pose):
            self.man_group.set_pose_target(goal_pose.pose)
            plan = self.man_group.plan()
            self.man_group.execute(plan, wait=True)

        else:
            rospy.logwarn("[PICK PLACE] Goal pose is not reachable!")

        # Ensures that there is no residual movement and clear the target
        self.man_group.stop()
        self.man_group.clear_pose_targets()

        curr_pose = self.man_group.get_current_pose()
        success = all_close(goal_pose, curr_pose, self.position_tolerance, self.orientation_tolerance)

        if not success:
            goal_position, goal_euler = pose_to_lists(goal_pose.pose, 'euler')
            curr_position, curr_euler = pose_to_lists(curr_pose.pose, 'euler')

            goal_position, goal_quat = pose_to_lists(goal_pose.pose, 'quaternion')
            curr_position, curr_quat = pose_to_lists(curr_pose.pose, 'quaternion')

            rospy.logwarn("Obtained pose is not sufficiently close to goal pose")
            rospy.logdebug("Goal quaternion orientation %s", goal_quat)
            rospy.logdebug("Current quaternion orientation %s", curr_quat)


            rospy.logdebug("Goal euler orientation %s", goal_euler)
            rospy.logdebug("Current euler orientation %s", curr_euler)
        return success

    def open_ee(self):
        rospy.logdebug("[PICK PLACE] Opening end effector")
        success = self.move_ee("Open")
        self.remove_attached_target_object()
        return True

    def close_ee(self):
        rospy.logdebug("[PICK PLACE] Closing end effector")
        success = self.move_ee("Closed")
        return True

    def home_man(self):
        rospy.logdebug("[PICK PLACE] Homeing manipulator")
        return self.move_to_joint_target(self.man_group, 'Upright')

    def move_ee(self, named_target):
        return self.move_to_joint_target(self.ee_group, named_target)

    def move_to_joint_target(self, group, named_target):
        group.set_named_target(named_target)

        plan = group.plan()
        group.execute(plan, wait=True)
        group.stop()

        # rospy.loginfo("Target joint values: %s", group.get_joint_value_target())
        # rospy.loginfo("Actual joint values: %s", group.get_current_joint_values())

        # success = all_close(group.get_joint_value_target(), group.get_current_joint_values(), self.position_tolerance, self.orientation_tolerance)
        success = True
        group.clear_pose_targets()
        return success

    def received_all_data(self):
        success = (self.EE_GRASP != None) and (self.grasp_pose != None) and (self.pre_grasp_pose != None) and (self.pre_place_pose != None) and (self.place_pose != None)
        return success

    def clear_all_data(self):
        self.EE_GRASP = None
        self.grasp_pose = None
        self.pre_grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None

    ### Log state update
    def log_state_update(self):
        rospy.loginfo("[PICK PLACE] updated pick place state, from %s to %s",
                      self.prev_state, self.state)

    def command_robot(self):

        # Update state state
        if (self.state != "init") and not self.received_all_data():
            self.prev_state = self.state
            self.state = "init"
            self.take_action()

        if self.state == "init" and self.received_all_data():
            self.prev_state = self.state
            self.state = "idle"
            self.take_action()

        if (self.state == "idle") and (self.goal_state == "pick"):
            self.prev_state = self.state
            self.state = self.goal_state # self.goal_state
            self.take_action()

        if (self.state == "pick") and (self.goal_state == "place"):
            self.prev_state = self.state
            self.state = self.goal_state # self.goal_state
            self.take_action()

        elif self.goal_state == "home":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif self.goal_state == "open":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif self.goal_state == "close":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()


    def take_action(self):
        self.log_state_update()
        msg = String()
        success = None

        # Take action based on state
        if self.state == "init":
            pass

        elif self.state == "idle":
            pass

        elif self.state == "pick":
            success = self.pick()

        elif self.state == "place":
            success = self.place()
            self.clear_all_data()

        elif self.state == "move":
            success = self.go_to_pre_grap_pose()

        elif self.state == "home":
            success = self.home_man()
            self.clear_all_data()

        elif self.state == "open":
            success = self.open_ee()
            self.state = "idle"
            self.clear_all_data()

        elif self.state == "close":
            success = self.close_ee()
            self.state = "idle"


        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.goal_state = None
            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Robot command failed")
                self.goal_state = None


            self.pub_e_out.publish(msg)
        else:
            # no robot command has been executed
            pass

def main():
    try:
        pp = Pick_Place()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            pp.command_robot()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
