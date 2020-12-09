#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import random
import numpy as np

# msgs
from flex_grasp.msg import FlexGraspErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from func.utils import add_pose
from flex_shared_resources.utils.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position, point_to_pose

import tf2_ros
import tf2_geometry_msgs

class TransformPose(object):

    def __init__(self, node_name):

        self.node_name = node_name

        self.simulation = rospy.get_param("robot_sim")
        self.robot_base_frame = rospy.get_param('robot_base_frame')
        self.planning_frame = rospy.get_param('planning_frame')

        # params
        self.surface_height = 0.019  # [m]
        self.peduncle_height = 0.070  # [m]

        # the sag_angle is used to take into account the sagging of the robot during operation
        if self.simulation:
            self.sag_angle = None
        else:
            self.sag_angle = np.deg2rad(6.0)

        # To determine the grasping height we need several dimensions of the manipulator
        height_finger = 0.040  # [m]
        finger_link2ee_link = 0.023  # [m]
        height_finger_tip = 0.007
        diameter_pedunlce = 0.004

        pre_grasp_distance = 0.04  # [m]
        grasp_height = height_finger + finger_link2ee_link - height_finger_tip - diameter_pedunlce
        pre_grasp_height = grasp_height + pre_grasp_distance

        grasp_xyz = [0, 0, grasp_height]           # [m]
        pre_grasp_xyz = [0, 0, pre_grasp_height]   # [m]
        place_xyz = grasp_xyz
        pre_place_xyz = pre_grasp_xyz
        ee_rpy = [0, np.pi/2, 0]                   # neutral rotation of ee relative to the robot base frame

        self.wrist_lower_limit = -np.pi/2
        self.wrist_upper_limit = np.pi/2

        # for generating a place pose
        self.r_min = 0.15
        self.r_max = 0.23
        self.x_min = 0.17

        # create dict which stores all poses
        keys = ['pre_grasp', 'grasp', 'pre_place', 'place']
        self.action_pose = dict.fromkeys(keys)

        # Initialize Publishers
        self.pose_pub = {}
        for key in self.action_pose:
            self.pose_pub[key] = rospy.Publisher(key + '_pose', PoseStamped, queue_size=5, latch=True)

        peduncle_height_pub = rospy.Publisher('peduncle_height', Float32, queue_size=5, latch=True)
        msg = Float32()
        msg.data = self.peduncle_height
        peduncle_height_pub.publish(msg)

        rospy.Subscriber("peduncle_height", Float32, self.peduncle_height_cb)
        random.seed(0)

        self.pose_transform = {'pre_grasp': point_to_pose(pre_grasp_xyz, ee_rpy),
                               'grasp':     point_to_pose(grasp_xyz, ee_rpy),
                               'pre_place': point_to_pose(pre_place_xyz, ee_rpy),
                               'place':     point_to_pose(place_xyz, ee_rpy)}

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def peduncle_height_cb(self, msg):
        self.peduncle_height = msg.data
        rospy.logdebug("[%s] Received new peduncle height: %s", self.node_name, self.peduncle_height)

    def get_object_height(self):
        """
            the object height, relative to the robot_base frame
        """
        return self.peduncle_height - self.surface_height

    def transform_current_object_pose(self, current_object_pose):
        """
            transform the current pose of the target object to the robot_base_frame
        """
        current_object_pose = self.transform_pose(current_object_pose, self.robot_base_frame)

        object_position, object_orientation = pose_to_lists(current_object_pose.pose, 'euler')

        # override the grasp height using the defined peduncle and surface height
        current_object_pose.pose.position = list_to_position((object_position[0], object_position[1], self.get_object_height()))
        current_object_pose = self.limit_wrist_angle(current_object_pose)
        return current_object_pose

    def get_target_object_pose(self, timeout=1):
        """
            Generate a random place pose
        """

        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < timeout) and not rospy.is_shutdown():
            theta = np.deg2rad(random.randrange(-90, 90, 1))  # [rad]
            r = self.r_min + random.random() * (self.r_max - self.r_min)  # [m]
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            if x >= self.x_min:
                break

        orientation = np.deg2rad(random.randrange(-180, 180, 1))  # [rad]

        goal_rpy = [0, 0, orientation]
        goal_xyz = [x, y, self.get_object_height()]

        target_object_pose = point_to_pose_stamped(goal_xyz, goal_rpy, self.robot_base_frame, rospy.Time.now())
        target_object_pose = self.limit_wrist_angle(target_object_pose)
        return target_object_pose

    def generate_action_poses(self, current_object_pose):
        """
            generates action poses for the pre_grasp, grasp, pre_place and place actions
        """

        if current_object_pose is None:
            rospy.logwarn("[%s] Cannot transform pose, since object_pose still empty!", self.node_name)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        current_object_pose = self.transform_current_object_pose(current_object_pose)
        target_object_pose = self.get_target_object_pose()

        # transform to planning frame
        current_object_pose = self.transform_pose(current_object_pose, self.planning_frame)
        target_object_pose = self.transform_pose(target_object_pose, self.planning_frame)

        action_pose = {}
        for key in self.pose_transform:
            added_pose = PoseStamped()
            if 'grasp' in key:
                pose_stamped = current_object_pose
            elif 'place' in key:
                pose_stamped = target_object_pose
            else:
                rospy.logwarn("[%s] Failed to add pose stamped: Unknown key", self.node_name)
                return FlexGraspErrorCodes.FAILURE

            added_pose.header.frame_id = pose_stamped.header.frame_id
            added_pose.header.stamp = pose_stamped.header.stamp
            added_pose.pose = add_pose(self.pose_transform[key], pose_stamped.pose)

            if added_pose is None:
                rospy.logwarn("[%s] Failed to add pose stamped", self.node_name)
                return FlexGraspErrorCodes.FAILURE
            else:
                action_pose[key] = added_pose

        for key in action_pose:
            rospy.logdebug("[%s] %s height is %s [m]", self.node_name, key, action_pose[key].pose.position.z)

        # compensate
        if self.sag_angle is not None:
            for key in action_pose:
                action_pose[key] = self.compensate_for_sagging(action_pose[key])

        self.action_pose = action_pose
        self.pub_all_poses()
        return FlexGraspErrorCodes.SUCCESS

    def pub_all_poses(self):
        for key in self.pose_pub:
            self.pose_pub[key].publish(self.action_pose[key])

    def compensate_for_sagging(self, pose_stamped):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        r = (x**2 + y**2)**0.5
        delta_height = np.tan(self.sag_angle)*r
        rospy.logdebug("[%s] Adding %s [m] height to due to radius %s [m]", self.node_name, delta_height, r)
        pose_stamped.pose.position.z += delta_height

    def limit_wrist_angle(self, object_pose):
        """
            limit orientation around z axis of a given pose_stamped, such that the desired wrist angle lies within the
            joint limits
        """
        # TODO: currently this method only works if the angle_wrist lies within a 180gedree range of the limits!

        if not object_pose.header.frame_id == self.robot_base_frame:
            rospy.logwarn("Cannot limit orientation, pose is not given with respect to %s frame", self.robot_base_frame)
            return object_pose

        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')

        angle_object = object_orientation[2]
        angle_base = np.arctan2(object_position[1], object_position[0])
        angle_wrist = angle_object - angle_base

        if not(self.wrist_lower_limit < angle_wrist < self.wrist_upper_limit):
            angle_object = angle_object + np.pi

        object_pose.pose.orientation = list_to_orientation([0, 0, angle_object])
        return object_pose


    def transform_pose(self, pose_stamped, to_frame):
        original_frame = pose_stamped.header.frame_id

        try:
            transform = self.tfBuffer.lookup_transform(to_frame, original_frame, time=rospy.Time.now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if transform is None:
            rospy.logwarn("[%s] Cannot transform pose, failed to lookup transform from %s to %s!", self.node_name,
                          original_frame, to_frame)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
