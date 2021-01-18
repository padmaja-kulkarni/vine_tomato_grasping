#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np

# msgs
from flex_grasp.msg import FlexGraspErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from func.utils import add_pose
from flex_shared_resources.utils.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position, point_to_pose

from flex_shared_resources.utils.pose_generator import PoseGenerator
from flex_shared_resources.data_logger import DataLogger
from flex_shared_resources.experiment_info import ExperimentInfo

import tf2_ros
import tf2_geometry_msgs

class TransformPose(object):

    def __init__(self, node_name, playback=False):

        self.node_name = node_name

        # state
        self.playback = playback
        if self.playback:
            rospy.loginfo("[{0}] Transform pose launched in playback mode!".format(self.node_name))

        self.simulation = rospy.get_param("robot_sim")
        self.robot_base_frame = rospy.get_param('robot_base_frame')
        self.planning_frame = rospy.get_param('planning_frame')

        # the sag_angle is used to take into account the sagging of the robot during operation
        if self.simulation:
            self.sag_angle = None
            self.peduncle_height = 0.070  # [m]
        else:
            self.sag_angle = np.deg2rad(7.0)
            self.peduncle_height = 0.080  # [m]

        # To determine the grasping height we need several dimensions of the manipulator
        self.surface_height = 0.019  # [m]
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

        self.wrist_limits = [-np.pi/2, np.pi/2]  # [lower_limit, upper_limit]

        # for generating a place pose
        self.pose_generator = PoseGenerator(r_range=[0.15, 0.23], x_min=0.17, frame=self.robot_base_frame,
                                            seed=self.node_name)

        self.exp_info = ExperimentInfo(self.node_name)

        # Initialize Publishers
        keys = ['pre_grasp', 'grasp', 'pre_place', 'place']
        topics_out = {}
        types_out = {}
        for key in keys:
            topics_out[key] = key + '_pose'
            types_out[key] = PoseStamped

        self.action_pose = dict.fromkeys(keys)
        self.output_logger = DataLogger(self.node_name, topics_out, types_out, bag_name=self.node_name)
        self.settings_logger = DataLogger(self.node_name, {"settings": "peduncle_height"},
                                          {"settings": Float32}, bag_name='peduncle_height')

        peduncle_height_pub = rospy.Publisher('peduncle_height', Float32, queue_size=5, latch=True)
        msg = Float32()
        msg.data = self.peduncle_height
        peduncle_height_pub.publish(msg)

        rospy.Subscriber("peduncle_height", Float32, self.peduncle_height_cb)

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
        """ return the object height, relative to the robot_base frame"""
        return self.peduncle_height - self.surface_height

    def transform_current_object_pose(self, current_object_pose):
        """ transform the current pose of the target object to the robot_base_frame"""
        current_object_pose = self.transform_pose(current_object_pose, self.robot_base_frame)

        object_position, object_orientation = pose_to_lists(current_object_pose.pose, 'euler')

        # override the grasp height using the defined peduncle and surface height
        current_object_pose.pose.position = list_to_position((object_position[0], object_position[1], self.get_object_height()))
        current_object_pose = self.limit_wrist_angle(current_object_pose)
        return current_object_pose

    def get_target_object_pose(self):
        """Generate a random place pose"""
        self.pose_generator.z = self.get_object_height()
        target_object_pose = self.pose_generator.generate_pose_stamped(seed=int(int(self.exp_info.id)*np.pi*10**2))
        target_object_pose = self.limit_wrist_angle(target_object_pose)
        return target_object_pose

    def generate_action_poses(self, current_object_pose):
        """
            generates action poses for the pre_grasp, grasp, pre_place and place actions
        """
        if self.playback:
            rospy.loginfo("[{0}] Playback is active: publishing messages from bag!".format(self.node_name))
            success = self.output_logger.publish_messages_from_bag(self.exp_info.path, self.exp_info.id)
            return success

        if current_object_pose is None:
            rospy.logwarn("[%s] Cannot transform pose, since object_pose still empty!", self.node_name)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        peduncle_height = Float32()
        peduncle_height.data = self.peduncle_height
        self.settings_logger.write_messages_to_bag({"settings": peduncle_height}, self.exp_info.path, self.exp_info.id)
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

        for key in action_pose:
            action_pose[key] = self.transform_pose(action_pose[key], self.robot_base_frame)

        self.action_pose = action_pose
        success = self.output_logger.publish_messages(self.action_pose, self.exp_info.path, self.exp_info.id)
        return success

    def compensate_for_sagging(self, pose_stamped):
        """
            The real robot bends under the weight of itself, therefore it may be desired to slightly increase the target
            pose height.
        """
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        r = (x**2 + y**2)**0.5
        delta_height = np.tan(self.sag_angle)*r
        rospy.logdebug("[%s] Adding %s [m] height to due to radius %s [m]", self.node_name, delta_height, r)
        pose_stamped.pose.position.z += delta_height
        return pose_stamped

    def limit_wrist_angle(self, object_pose):
        """
            limit orientation around z axis of a given pose_stamped, such that the desired wrist angle lies within the
            joint limits
        """
        # TODO: currently this method only works if the angle_wrist lies within a 180degree range of the limits!

        if not object_pose.header.frame_id == self.robot_base_frame:
            rospy.logwarn("Cannot limit orientation, pose is not given with respect to %s frame", self.robot_base_frame)
            return object_pose

        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')

        angle_object = object_orientation[2]
        angle_base = np.arctan2(object_position[1], object_position[0])
        angle_wrist = angle_object - angle_base

        if not(self.wrist_limits[0] < angle_wrist < self.wrist_limits[1]):
            angle_object = angle_object + np.pi
            
        object_pose.pose.orientation = list_to_orientation([0, 0, angle_object])
        return object_pose
    
    def transform_pose(self, pose_stamped, to_frame):
        original_frame = pose_stamped.header.frame_id

        try:
            transform = self.tfBuffer.lookup_transform(to_frame, original_frame, time=rospy.Time.now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[%s] Cannot transform pose, failed to lookup transform from %s to %s!", self.node_name,
                          original_frame, to_frame)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
