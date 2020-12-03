#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

from moveit_commander import MoveItCommanderException
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# messages
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from flex_grasp.msg import ImageProcessingSettings

def pose_to_lists(pose_msg, orientation_type):
    """
        if orientation_type == euler, retrun euler angles in radians
    """
    # print(orientation_type)
    pose = pose_to_list(pose_msg)
    position = pose[0:3]
    quaternion = pose[3:7]

    if orientation_type == "euler":
        euler = euler_from_quaternion(quaternion)
        orientation = euler
    elif orientation_type == "quaternion":
        orientation = quaternion
    else:
        raise MoveItCommanderException("Unknown type, accepts type 'euler' or 'quaternion'")

    return position, orientation


def position_to_list(position_msg):
    position = []

    position.append(position_msg.x)
    position.append(position_msg.y)
    position.append(position_msg.z)

    return position

def list_to_position(position_list):
    position_msg = Point()

    if len(position_list) == 3:
        position_msg.x = position_list[0]
        position_msg.y = position_list[1]
        position_msg.z = position_list[2]
    else:
        raise MoveItCommanderException("Expected 3 elements in list: (x,y,z)")

    return position_msg


def orientation_to_list(orientation_msg):
    orientation = []

    orientation.append(orientation_msg.x)
    orientation.append(orientation_msg.y)
    orientation.append(orientation_msg.z)
    orientation.append(orientation_msg.w)

    return orientation

def list_to_orientation(orientation_list):

    if len(orientation_list) == 3:
        quat_list = quaternion_from_euler(orientation_list[0], orientation_list[1], orientation_list[2])
    elif len(orientation_list) == 4:
        quat_list = orientation_list
    else:
        raise MoveItCommanderException("Expected orinetation list containing 3 (x, y, z) or 4 (x, y, z, w) elements")

    orientation_msg = Quaternion()
    orientation_msg.x = quat_list[0]
    orientation_msg.y = quat_list[1]
    orientation_msg.z = quat_list[2]
    orientation_msg.w = quat_list[3]

    return orientation_msg

def point_to_pose_stamped(xyz, rpy, frame, time):

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = time

    quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    pose_stamped.pose.position.x = xyz[0]
    pose_stamped.pose.position.y = xyz[1]
    pose_stamped.pose.position.z = xyz[2]

    return pose_stamped


def point_to_pose(xyz, rpy):
    pose = Pose()

    quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose.position.x = xyz[0]
    pose.position.y = xyz[1]
    pose.position.z = xyz[2]

    return pose


def settings_lib_to_msg(lib):
    msg = ImageProcessingSettings()

    tom_lib = lib['detect_tomato']
    pend_lib = lib['detect_peduncle']
    grasp_lib = lib['compute_grasp']

    # distances in px
#    msg.tomato_radius_min_frac = tom_lib['radius_min_frac']
#    msg.tomato_radius_max_frac = tom_lib['radius_max_frac']
#    msg.tomato_distance_min_frac = tom_lib['distance_min_frac']

    # distances in px
    msg.tomato_radius_min_mm = tom_lib['radius_min_mm']
    msg.tomato_radius_max_mm = tom_lib['radius_max_mm']
    # msg.tomato_distance_min_mm = tom_lib['distance_min_mm']

#    msg.dp = tom_lib['dp']
    msg.param1 = tom_lib['param1']
    msg.param2 = tom_lib['param2']
#    msg.blur_size = tom_lib['blur_size']
#    msg.ratio_threshold = tom_lib['ratio_threshold']

#    msg.branch_length_min_px = pend_lib['branch_length_min_px']
    msg.branch_length_min_mm = pend_lib['branch_length_min_mm']

    msg.grasp_length_min_mm = grasp_lib['grasp_length_min_mm']

    return msg

def settings_msg_to_lib(msg):

    tom_lib = {}

    # distances in px
#    tom_lib['radius_min_frac'] = msg.tomato_radius_min_frac
#    tom_lib['radius_max_frac'] = msg.tomato_radius_max_frac
#    tom_lib['distance_min_frac'] = msg.tomato_distance_min_frac

    # distances in mm
    tom_lib['radius_min_mm'] = msg.tomato_radius_min_mm
    tom_lib['radius_max_mm'] = msg.tomato_radius_max_mm
    # tom_lib['distance_min_mm'] = msg.tomato_distance_min_mm

#    tom_lib['dp'] = msg.dp
    tom_lib['param1'] = msg.param1
    tom_lib['param2'] = msg.param2
#    tom_lib['blur_size'] = msg.blur_size
#    tom_lib['ratio_threshold'] = msg.ratio_threshold


    pend_lib = {}
#    pend_lib['branch_length_min_px'] = msg.branch_length_min_px
    pend_lib['branch_length_min_mm'] = msg.branch_length_min_mm

    grasp_lib = {}
    grasp_lib['grasp_length_min_mm'] = msg.grasp_length_min_mm

    lib = {'detect_tomato': tom_lib, 'detect_peduncle': pend_lib, 'compute_grasp': grasp_lib}
    return lib
