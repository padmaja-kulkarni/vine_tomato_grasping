#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

import cv2
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose

from flex_shared_resources.utils.conversions import pose_to_lists
from moveit_commander.conversions import list_to_pose
import pyrealsense2 as rs


def wait_for_param(param, timeout):
    """return parameter if it is present on the parameter server within timeout"""
    start_time = rospy.get_time()

    while (rospy.get_time() - start_time < timeout):

        if rospy.has_param(param):
            return rospy.get_param(param)

    rospy.logwarn("Parameter %s can not be loaded from parameter server: timeout passed", param)
    return None

def camera_info2rs_intrinsics(camera_info):

    # init object
    rs_intrinsics = rs.intrinsics()

    # dimensions
    rs_intrinsics.width = camera_info.width
    rs_intrinsics.height = camera_info.height

    # principal point coordinates
    rs_intrinsics.ppx = camera_info.K[2]
    rs_intrinsics.ppy = camera_info.K[5]

    # focal point
    rs_intrinsics.fx = camera_info.K[0]
    rs_intrinsics.fy = camera_info.K[4]

    return rs_intrinsics


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

        # check position
        diff = np.abs(pos_goal - pos_actual)
        position_close = np.all(diff < position_tol)

        # https://math.stackexchange.com/a/90098
        dist = 2*np.inner(q_goal, q_actual)**2 - 1
        diff = abs(np.arccos(dist))
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
    """
        Add two stamped poses
    """
    pose_stamp_c = PoseStamped()

    if pose_stamp_a.header.frame_id == pose_stamp_b.header.frame_id:
        pose_stamp_c.header = pose_stamp_a.header
        pose_stamp_c.pose = add_pose(pose_stamp_a.pose, pose_stamp_b.pose)
        return pose_stamp_c
    else:
        # TODO: add raise exception
        return None


def colored_depth_image(depth_image, min_dist=0.4, max_dist=0.6):

    # remove outliers
    depth_image[depth_image > max_dist] = max_dist
    depth_image[depth_image < min_dist] = min_dist

    max_val = 255
    min_val = 0

    norm_depth_image = (depth_image - min_dist)/(max_dist - min_dist) * (max_val - min_val) + min_val
    norm_depth_image = np.uint8(norm_depth_image)

    return cv2.applyColorMap(norm_depth_image, cv2.COLORMAP_JET)

