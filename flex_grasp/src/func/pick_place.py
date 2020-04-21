#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 09:23:15 2020

@author: taeke
"""


from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation, Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

self.GRIPPER_EFFORT = [100.0]

def pick(self):
    """ Pick an object
    """

    rospy.logdebug("==STARTING PICK PROCEDURE===")

    # Initialize grasp
    grasps = Grasp()

    # Pre grasp posture
    time_list = [0.5]
    grasps.pre_grasp_posture = self.make_gripper_posture(self.EE_OPEN, time_list)

    # Grasp posture
    time_list = [1.5]
    grasps.grasp_posture = self.make_gripper_posture(self.EE_GRASP, time_list)

    # Set the approach and retreat parameters as desired
    grasps.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1, [0, 0, -1.0])
    grasps.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, [0, 0, 1.0])

    # grasp pose
    grasps.grasp_pose = self.grasp_pose

    # touchable obejcts
    grasps.allowed_touch_objects = ['table', 'wall', 'tomato_0', 'tomato_1', 'peduncle']

    self.man_group.set_support_surface_name("table")

    # Pick
    result = None
    n_attempts = 0

    while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_pick_attempts:

        n_attempts += 1
        rospy.loginfo("Pick attempt: " +  str(n_attempts))
        result = self.man_group.pick(self.target_object_name, grasps)

        rospy.sleep(0.2)

    return result == MoveItErrorCodes.SUCCESS

def place(self):

    # place
    result = None
    n_attempts = 0

    # Repeat until we succeed or run out of attempts
    while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_place_attempts:
        n_attempts += 1
        rospy.loginfo("Place attempt: " +  str(n_attempts))

        result = self.man_group.place(self.target_object_name) # self.place_pose
        rospy.sleep(0.2)

    return result == MoveItErrorCodes.SUCCESS

rospy.logdebug("==STARTING PLACE PROCEDURE===")

def make_gripper_posture(self, joint_positions, time_list):
    # Initialize the joint trajectory for the gripper joints
    t = JointTrajectory()

    # Set the joint names to the gripper joint names
    t.joint_names = self.ee_joint_names

    for time in time_list:

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
