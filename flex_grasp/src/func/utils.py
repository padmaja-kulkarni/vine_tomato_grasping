#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

from func.conversions import pose_to_lists
from moveit_commander.conversions import list_to_pose
import pyrealsense2 as rs
from math import pi

def camera_info2intrinsics(camera_info):

    # init object
    intrin = rs.intrinsics()

    # dimensions
    intrin.width = camera_info.width
    intrin.height = camera_info.height

    # principal point coordinates
    intrin.ppx = camera_info.K[2]
    intrin.ppy = camera_info.K[5]

    # focal point
    intrin.fx = camera_info.K[0]
    intrin.fy = camera_info.K[4]

    return intrin


def pose_close(goal, actual, position_tolerance, orientation_tolerance):
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
        return pose_close(goal.pose, actual.pose, position_tolerance, orientation_tolerance)

    elif type(goal) is Pose:
        return pose_close(pose_to_list(goal), pose_to_list(actual), position_tolerance, orientation_tolerance)

    return True

def joint_close(goal, actual, angle_tolerance):
    is_close = True    
    
    for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > angle_tolerance:
                is_close = False

    return is_close

def add_pose_stamped(pose_stamp_a, pose_stamp_b):
    pose_stamp_c = PoseStamped()
    
    if pose_stamp_a.header.frame_id == pose_stamp_b.header.frame_id:
        pose_stamp_c.header.frame_id = pose_stamp_a.header.frame_id
    else:
        print("frame_id's do not match, returning an empty pose stamped")
        return pose_stamp_c
    
    position_a, orientation_a = pose_to_lists(pose_stamp_a.pose, 'euler')
    position_b, orientation_b = pose_to_lists(pose_stamp_b.pose, 'euler')
    
    position_c = add_lists(position_a, position_b)
    orientation_c = add_lists(orientation_a, orientation_b)
    
    pose_c = list_to_pose(position_c + orientation_c)
    pose_stamp_c.pose = pose_c
    
    return pose_stamp_c

def add_lists(list1, list2):
    return [sum(x) for x in zip(list1, list2)]

def multiply_lists(list1, list2):
    return [x*y for x, y in zip(list1, list2)]


def neg_list(list1):
    return [ -x for x in list1]

def deg2rad(deg):
    return float(deg)/180.0*pi