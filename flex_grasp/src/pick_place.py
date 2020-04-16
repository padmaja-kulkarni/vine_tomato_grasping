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
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation, Grasp, GripperTranslation

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from std_msgs.msg       import Float64


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

        self.initialise_robot()
        self.initialise_enviroment()


        self.GRIPPER_EFFORT = [1.0]


        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.event = None

        # Subscribers
        rospy.Subscriber("endEffectorPose", PoseStamped, self.ee_pose_cb)
        rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers

        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=True)

    def ee_distance_cb(self, msg):
        dist = msg.data
        # if self.GRIPPER_GRASP is None:
        #     dist = msg.data
            # self.GRIPPER_GRASP = add_list(self.EE_CLOSED, [dist/2, -dist/2])
            # rospy.logdebug("[PICK PLACE] Received newend effector distance message")

    def ee_pose_cb(self, msg):
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
        manipulator_group_name = rospy.get_param('manipulator_group_name')
        ee_group_name = rospy.get_param('ee_group_name')

        manipulator_group = moveit_commander.MoveGroupCommander(manipulator_group_name)
        ee_group = moveit_commander.MoveGroupCommander(ee_group_name)

        eef_link = manipulator_group.get_end_effector_link()

        manipulator_joint_names = manipulator_group.get_joints()
        # ee_joint_names = ee_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys() # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()
        ee_group.clear_pose_target

        rospy.logdebug("Joint names: %s", ee_joint_names)
        rospy.logdebug("EE_CLOSED: %s", EE_CLOSED)
        rospy.logdebug("EE_OPEN: %s", EE_OPEN)

        # Allow replanning to increase the odds of a solution
        manipulator_group.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        manipulator_group.set_planning_time(5)

        # Allow some leeway in position (meters) and orientation (radians)
        manipulator_group.set_goal_position_tolerance(0.05)
        manipulator_group.set_goal_orientation_tolerance(0.1)

        ee_group.set_goal_position_tolerance(0.05)
        ee_group.set_goal_orientation_tolerance(0.1)

        self.max_pick_attempts = 10

        self.manipulator_group_name = manipulator_group_name
        self.ee_group_name = ee_group_name
        self.robot = robot
        self.group = manipulator_group
        self.robot_goal_pose = None
        self.eef_link = eef_link
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
        self.scene.remove_attached_object(self.eef_link, self.target_object_name)

        rospy.sleep(1)

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

        request.ik_request.group_name = self.manipulator_group_name
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)

        if not response.error_code.val == MoveItErrorCodes.SUCCESS:
            return False
        else:
            return True

    def pick(self):
        """ Pick an object
        """

        rospy.logdebug("==STARTING PICK PROCEDURE===")

        # Initialize grasp
        grasps = Grasp()

        # Pre grasp posture
        grasps.pre_grasp_posture = self.make_gripper_posture(self.EE_OPEN, 0.5)

        # Grasp posture
        rospy.logdebug("Grasping gripper posture: %s", self.EE_CLOSED)
        grasps.grasp_posture = self.make_gripper_posture(self.EE_CLOSED, 1.0)

        # Set the approach and retreat parameters as desired
        grasps.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [0, 0, -1.0])
        grasps.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0, 0, 1.0])

        # grasp pose
        grasps.grasp_pose = self.robot_goal_pose

        # touchable obejcts
        grasps.allowed_touch_objects = ['table', 'wall', 'tomato_0', 'tomato_1', 'peduncle']

        self.group.set_support_surface_name("table")

        # Pick
        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_pick_attempts:

            # the object cannot be attached before attempting an grasp
            self.scene.remove_attached_object(self.eef_link, self.target_object_name)

            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = self.group.pick(self.target_object_name, grasps)
            # rospy.logdebug("Resulting MoveItErrorCode: %s", result)
            rospy.sleep(0.2)

        self.robot_goal_pose = None

        if result == MoveItErrorCodes.SUCCESS:
            return True
        else:
            return False

    def make_gripper_posture(self, joint_positions,time):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = self.ee_joint_names

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = self.GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(time)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = "world"

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g


    def go_to_pose_goal(self):
        """ plan and move to a pose goal


        """
        rospy.logdebug("==STARTING GO TO POSE GOAL PROCEDURE===")

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


        if ((self.robot_goal_pose is not None and ((self.event == "e_pick_place") or (self.event == "e_move"))) or self.event == "e_home"):
            msg = String()
            success = None

            if (self.event == "e_start") or (self.event == "e_pick_place"):
                succes = self.pick()
            elif self.event == "e_move":
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

def add_list(list1, list2):
    return [sum(x) for x in zip(list1, list2)]

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
