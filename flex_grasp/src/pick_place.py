#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

import sys
import rospy
import moveit_commander
import math as m
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation, Grasp

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest



# custom functions
from func.all_close import all_close

# enum
# from RobotState import RobotState


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

        self.initialise_enviroment()
        self.initialise_robot()

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.event = None

        # Subscribers
        rospy.Subscriber("endEffectorPose", PoseStamped, self.pose_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers

        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=True)


    def pose_cb(self, msg):
        if self.robot_goal_pose is None:
            self.robot_goal_pose = msg
            rospy.logdebug("[PICK PLACE] Received new move robot pose message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[PICK PLACE] Received new move robot event in message: %s", self.event)


    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        group_name = rospy.get_param('move_group_name')
        group = moveit_commander.MoveGroupCommander(group_name)

        # rospy.sleep(10)required?

        self.robot = robot
        self.group = group
        self.robot_goal_pose = None
        # self.state = RobotState.INITIALIZING

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name= ''):
        """ Wait until we see the changes reflected in the scene

            Args:

            Returns: True if the box was succesfully added, False otherwise.
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects()
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            known_objects = self.scene.get_known_object_names()
            rospy.logdebug("Known objects: %s", known_objects)

            is_known = box_name in known_objects

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4, box_name = ''):
        """ create a box with a given name.

        Args:

        Returns: True if the box was succesfully added, False otherwise.

        """
        box_pose = PoseStamped()
        box_pose.header.frame_id =  self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.y = 1.0

        # Add box
        self.scene.add_box(box_name, box_pose, size=(1, 1, 0.1))

        # Check if box has been added
        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name = box_name)

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

        group_name = rospy.get_param('move_group_name')
        request.ik_request.group_name = group_name
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)

        if not response.error_code.val == MoveItErrorCodes.SUCCESS:
            return False
        # We do not need the actual solution
        # joint_state = response.solution.joint_state
        # for name, position in zip(joint_state.name, joint_state.position):
        #     if name in ArmJoints.names():
        #         rospy.loginfo('{}: {}'.format(name, position))
        return True


    def initialise_enviroment(self):
        """" Checks wether the RViz enviroment is correctly set


        """

        rospy.logdebug("===INITIALIZING ENVIROMENT====")

        # interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(0.1)

        # Check wether table and wall objects are present
        known_objects_prev = None
        while True:
            known_objects = self.scene.get_known_object_names()

            if not known_objects == known_objects_prev:
                known_objects_prev = known_objects
                rospy.logdebug("Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("Table and wall object not present, refusing to continue before they are added")
            rospy.sleep(0.1)


#        if self.add_box(box_name = 'table'):
#            rospy.logdebug("Succesfully added table")
#        else:
#            rospy.logwarn("Unable to add table")
#
#        rospy.logdebug( "Known objects: %s", self.scene.get_known_object_names())

    def pick(self):
        """ Pick an object
        """

        rospy.logdebug("==STARTING PICK PROCEDURE===")

        grasps = Grasp()

        grasps.grasp_pose.header.frame_id = "world"

        grasps.grasp_pose.pose = self.robot_goal_pose.pose


        # Setting pre-grasp approach
        grasps.pre_grasp_approach.direction.header.frame_id = "world"
        grasps.pre_grasp_approach.direction.vector.z = 1.0
        grasps.pre_grasp_approach.min_distance = 0.005
        grasps.pre_grasp_approach.desired_distance = 0.25

        # Setting post-grasp retreat
        grasps.post_grasp_retreat.direction.header.frame_id = "world"
        grasps.post_grasp_retreat.direction.vector.z = 1.0
        grasps.post_grasp_retreat.min_distance = 0.005
        grasps.post_grasp_retreat.desired_distance = 0.25

        rospy.logdebug("==SETTING PREGRASP POSTURE===")
        self.openGripper(grasps.pre_grasp_posture)

        rospy.logdebug("==SETTING GRASP POSTURE===")
        self.closedGripper(grasps.grasp_posture)

        self.group.set_support_surface_name("table")

        # self.group.

        rospy.logdebug("==PERFORM GRASP===")
        self.group.pick("tomato", grasps)

    def openGripper(self, posture):
        posture.joint_names = ["sdh_finger_11_joint", "sdh_knuckle_joint",
                               "sdh_finger_12_joint", "sdh_finger_13_joint",
                               "sdh_finger_21_joint", "sdh_finger_22_joint",
                               "sdh_finger_23_joint", "sdh_thumb_2_joint",
                               "sdh_thumb_3_joint"]

        jtp = JointTrajectoryPoint()
        jtp.positions = [0, -m.pi/2, 0, 0, -m.pi/2, 0, m.pi/2, 0]
        jtp.time_from_start = rospy.Duration(0.5)

        posture.points = [jtp]

    def closedGripper(self, posture):
        posture.joint_names = ["sdh_finger_11_joint", "sdh_knuckle_joint",
                               "sdh_finger_12_joint", "sdh_finger_13_joint",
                               "sdh_finger_21_joint", "sdh_finger_22_joint",
                               "sdh_finger_23_joint", "sdh_thumb_2_joint",
                               "sdh_thumb_3_joint"]

        jtp = JointTrajectoryPoint()
        jtp.positions = [0, 0, 0, 0, 0, 0, 0, 0]
        jtp.time_from_start = rospy.Duration(0.5)

        posture.points = [jtp]


    def go_to_pose_goal(self):
        """ plan and move to a pose goal


        """

        ## Only if inverse kinematics exist
        if self.compute_ik(self.robot_goal_pose):
            rospy.loginfo("Goal pose is reachable.")

            self.group.set_pose_target(self.robot_goal_pose.pose)
            plan = self.group.plan()
            self.group.execute(plan, wait=True)

        else:
            rospy.logwarn("Goal pose is not reachable!")


        # Ensures that there is no residual movement
        self.group.stop()

        # clear targets after planning with poses.
        self.group.clear_pose_targets()
        self.robot_goal_pose = None

        # verify goal pose is obtained
        return all_close(self.robot_goal_pose, self.group.get_current_pose(), 0.02)

    def go_to_home(self):
        """ Plan and move to home

        """
        self.group.set_named_target('Upright')

        plan = self.group.plan()
        self.group.execute(plan, wait=True)

        # Ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()

        return all_close(self.group.get_joint_value_target(), self.group.get_current_joint_values(), 0.02)


    def command_robot(self):


        if (self.robot_goal_pose is not None and self.event == "e_start") or self.event == "e_home":
            msg = String()
            success = None

            if self.event == "e_start":
                succes = self.go_to_pose_goal()
            elif self.event == "e_home":
                succes = self.go_to_home()

            if succes:
                msg.data = "e_success"
            else:
                msg.data = "e_failure"
                rospy.logwarn("Goal Tolerance Violated")

            self.event = None
            self.pub_e_out.publish(msg)

        else:
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
