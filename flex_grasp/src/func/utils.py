#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion

def all_close(goal, actual, position_tolerance, orientation_tolerance):
    """
    Convenience method for testing if a list of values are within a position_tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: position_tolerance  A float
    @returns: bool
    """

    if type(goal) is list:

        actual_position = actual[0:3]
        actual_quat = actual[3:7]

        goal_position = goal[0:3]
        goal_quat = goal[3:7]

        position_close = True

        # check position
        for index in range(len(goal_position)):
            if abs(actual_position[index] - goal_position[index]) > position_tolerance:
                position_close = False

        # check orientation
        orientation_close = True

        # actual_quat = goal_quat
        for index in range(len(goal_quat)):
            if abs(actual_quat[index] - goal_quat[index]) > orientation_tolerance:
                orientation_close = False

        # actual_quat = -goal_quat
        if not orientation_close:
            goal_quat = neg_list(goal_quat)

            for index in range(len(goal_quat)):
                if abs(actual_quat[index] - goal_quat[index]) > orientation_tolerance:
                    orientation_close = False
                else:
                    orientation_close = True

        return orientation_close, position_close

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, position_tolerance, orientation_tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), position_tolerance, orientation_tolerance)

    return True

def add_lists(list1, list2):
    return [sum(x) for x in zip(list1, list2)]

def multiply_lists(list1, list2):
    return [x*y for x, y in zip(list1, list2)]


def neg_list(list1):
    return [ -x for x in list1]
