#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

import cv2
import numpy as np

from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

from flex_shared_resources.utils.conversions import pose_to_lists
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


def pose_close(goal, actual, position_tol, orientation_tol):
    """
    Convenience method for testing if a list of values are within a position_tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: position_tolerance  A float
    @returns: bool
    """

    if type(goal) is list:
        pos_actual = np.array(actual[0:3])
        q_actual = np.array(actual[3:7])

        pos_goal = np.array(goal[0:3])
        q_goal = np.array(goal[3:7])

        position_close = True

        # check position
        diff = np.abs(pos_goal - pos_actual)
        # print(diff)
        position_close = np.all(diff < position_tol)

        dist = 2*np.inner(q_goal,q_actual)**2 - 1
        diff = abs(np.arccos(dist))
        # print(diff)
        orientation_close = np.all(diff < orientation_tol)

        return position_close, orientation_close

    elif type(goal) is PoseStamped:
        return pose_close(goal.pose, actual.pose, position_tol, orientation_tol)

    elif type(goal) is Pose:
        return pose_close(pose_to_list(goal), pose_to_list(actual), position_tol, orientation_tol)

    return True

def joint_close(goal_state, actual_state, angle_tolerance):
    is_close = True

    for goal_angle, actual_angle in zip(goal_state, actual_state):
            if abs(goal_angle - actual_angle) > angle_tolerance:
                is_close = False

    return is_close


def add_pose(pose_a, pose_b):
    """
        Add two poses
    """

    position_a, orientation_a = pose_to_lists(pose_a, 'euler')
    position_b, orientation_b = pose_to_lists(pose_b, 'euler')

    position_c = [a + b for a, b in zip(position_a, position_b)]
    orientation_c = [a + b for a, b in zip(orientation_a, orientation_b)]

    pose_c = list_to_pose(position_c + orientation_c)

    return pose_c


def add_pose_stamped(pose_stamp_a, pose_stamp_b):
    pose_stamp_c = PoseStamped()

    if pose_stamp_a.header.frame_id == pose_stamp_b.header.frame_id:
        pose_stamp_c.header = pose_stamp_a.header
    else:
        # TODO: add raise exception
        return None

    position_a, orientation_a = pose_to_lists(pose_stamp_a.pose, 'euler')
    position_b, orientation_b = pose_to_lists(pose_stamp_b.pose, 'euler')

    position_c = [a + b for a, b in zip(position_a, position_b)]
    orientation_c = [a + b for a, b in zip(orientation_a, orientation_b)]

    pose_c = list_to_pose(position_b + orientation_c)
    pose_stamp_c.pose = pose_c

    return pose_stamp_c

def add_lists(list1, list2):
    return [sum(x) for x in zip(list1, list2)]

def multiply_lists(list1, list2):
    return [x*y for x, y in zip(list1, list2)]


def neg_list(list1):
    return [ -x for x in list1]


def colored_depth_image(depth_image, min_dist = 0.4, max_dist = 0.6):

    # remove outliers
    depth_image[depth_image > max_dist] = max_dist
    depth_image[depth_image < min_dist] = min_dist

    max_val = 255
    min_val = 0

    norm_depth_image = (depth_image - min_dist)/(max_dist - min_dist) * (max_val - min_val) + min_val
    norm_depth_image = np.uint8(norm_depth_image)

    colored_depth_image = cv2.applyColorMap(norm_depth_image, cv2.COLORMAP_JET)

    return colored_depth_image
