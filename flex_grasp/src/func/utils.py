#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

import cv2

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


def pose_close(goal, actual, position_tol, orientation_tol):
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
        for (pos_goal, pos_act) in zip(goal_position, actual_position):
            position_difference = abs(pos_goal - pos_act)
            if abs(position_difference) > position_tol:
                # print(position_difference)
                position_close = False

        # check orientation
        orientation_close_1 = True # actual_quat = goal_quat
        orientation_close_2 = True # actual_quat = -goal_quat

        # actual_quat = goal_quat
        for (q_goal, q_actual) in zip(goal_quat, actual_quat):
            orientation_diff_1 = abs(q_goal - q_actual)
            orientation_diff_2 = abs(q_goal + q_actual)
            if orientation_diff_1 > orientation_tol:
                orientation_close_1 = False
            if orientation_diff_2 > orientation_tol:
                orientation_close_2 = False
                
            if orientation_diff_1 > orientation_tol and orientation_diff_2 > orientation_tol:
                pass
                # print(orientation_diff_1)
                # print(orientation_diff_2)
               
        orientation_close = orientation_close_1 or orientation_close_2

        return orientation_close, position_close

    elif type(goal) is PoseStamped:
        return pose_close(goal.pose, actual.pose, position_tol, orientation_tol)

    elif type(goal) is Pose:
        return pose_close(pose_to_list(goal), pose_to_list(actual), position_tol, orientation_tol)

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
    

def colored_depth_image(depth_image, min_dist = 0.4, max_dist = 0.7):
    
    # remove outliers
    _, depth_image = cv2.threshold(depth_image, max_dist, 0.0, cv2.THRESH_TRUNC)
    _, depth_image = cv2.threshold(-depth_image, -min_dist, 0.0, cv2.THRESH_TRUNC)
    depth_image = -depth_image
    # _, depth_image = cv2.threshold(depth_image, max_dist, 0.0, cv2.THRESH_TOZERO_INV)
    
    norm_depth_image = cv2.normalize(depth_image, None, 
                                     alpha=0, beta=255, 
                                     norm_type=cv2.NORM_MINMAX, 
                                     dtype=cv2.CV_8UC1)
    
    # eq_depth_image = cv2.equalizeHist(norm_depth_image)
    colored_depth_image = cv2.applyColorMap(norm_depth_image, cv2.COLORMAP_JET)
    
    return colored_depth_image
