#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 09:55:50 2020

@author: taeke
"""

import rospy
import random
import numpy as np

# msgs
from std_msgs.msg import String, Float32
from flex_grasp.msg import Truss
from flex_grasp.msg import FlexGraspErrorCodes
from geometry_msgs.msg import PoseStamped

from func.ros_utils import wait_for_variable, get_transform
from func.utils import add_pose
from flex_shared_resources.utils.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position, point_to_pose
from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log

import tf2_ros
import tf2_geometry_msgs


class TransformPose(object):
    node_name = 'TRANSFORM POSE'
    frequency = 10  # [hz]

    def __init__(self):

        self.debug_mode = rospy.get_param("transform_pose/debug")

        # initialize node
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[%s] Launching transform pose node in debug mode", self.node_name)
        else:
            log_level = rospy.INFO

        rospy.init_node("transform_pose", anonymous=True, log_level=log_level)
        self.rate = rospy.Rate(self.frequency)
        self.simulation = rospy.get_param("robot_sim")

        # params
        self.surface_height = 0.019  # [m]
        self.peduncle_height = 0.080  # [m]

        # the sag_angle is used to take into account the sagging of the robot during operation
        if self.simulation:
            self.sag_angle = 0
        else:
            self.sag_angle = np.deg2rad(6.0)

        # To determine the grasping height we need several dimensions of the menipulator
        height_finger = 0.040  # [m]
        finger_link2ee_link = 0.023  # [m]
        height_finger_tip = 0.007
        diameter_pedunlce = 0.004
        delta = 0.08  # [m] I had to add this additional height for simulation

        pre_grasp_distance = 0.04  # [m]
        self.grasp_height = height_finger + finger_link2ee_link - height_finger_tip - diameter_pedunlce + delta
        self.pre_grasp_height = self.grasp_height + pre_grasp_distance

        grasp_xyz = [0, 0, self.grasp_height]  # [m]
        pre_grasp_xyz = [0, 0, self.pre_grasp_height]  # [m]
        self.grasp_rpy = [-np.pi, np.pi/2, 0]

        self.wrist_lower_limit = -np.pi/2
        self.wrist_upper_limit = np.pi/2

        # for generating a place pose
        self.r_min = 0.15
        self.r_max = 0.23
        self.x_min = 0.17

        # params from server
        self.robot_base_frame = rospy.get_param('robot_base_frame')
        self.planning_frame = rospy.get_param('planning_frame')

        self.object_features = None
        self.command = None

        peduncle_height_pub = rospy.Publisher('peduncle_height', Float32, queue_size=5, latch=True)
        msg = Float32()
        msg.data = self.peduncle_height
        peduncle_height_pub.publish(msg)

        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("object_features", Truss, self.object_features_cb)
        rospy.Subscriber("peduncle_height", Float32, self.peduncle_height_cb)

        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

        # create dict which stores all poses
        keys = ['pre_grasp', 'grasp', 'pre_place', 'place']
        self.action_pose = dict.fromkeys(keys)

        # Initialize Publishers
        self.pose_pub = {}
        for key in self.action_pose:
            self.pose_pub[key] = rospy.Publisher(key + '_pose', PoseStamped, queue_size=5, latch=True)

        random.seed(0)

        self.pose_transform = {'pre_grasp': point_to_pose(pre_grasp_xyz, self.grasp_rpy),
                               'grasp': point_to_pose(grasp_xyz, self.grasp_rpy)}

        # Tranform
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[%s] Received command in message: %s", self.node_name, self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)

    def object_features_cb(self, msg):
        self.object_features = msg
        rospy.logdebug("[%s] Received new object feature message", self.node_name)

    def peduncle_height_cb(self, msg):
        self.peduncle_height = msg.data
        rospy.logdebug("[%s] Received new peduncle height: %s", self.node_name, self.peduncle_height)
        if self.object_features is not None:
            self.transform_pose()

    def generate_place_pose(self):  # pre_place_height, place_height
        """
            Generate a random place pose
        """
        x = 0.0
        while x <= self.x_min:
            theta = np.deg2rad(random.randrange(-90, 90, 1))  # [rad]
            r = self.r_min + random.random() * (self.r_max - self.r_min)  # [m]

            x = r * np.cos(theta)
            y = r * np.sin(theta)

        orientation = np.deg2rad(random.randrange(0, 360, 1))  # + theta  # [rad]

        pre_place_height = self.peduncle_height - self.surface_height + self.pre_grasp_height
        place_height = self.peduncle_height - self.surface_height + self.grasp_height

        place_rpy = [self.grasp_rpy[0], self.grasp_rpy[1], orientation]
        pre_place_xyz = [x, y, pre_place_height]
        place_xyz = [x, y, place_height]

        frame = self.robot_base_frame  # robot_base_frame
        time = rospy.Time.now()

        place_pose = {}
        place_pose['pre_place'] = point_to_pose_stamped(pre_place_xyz, place_rpy, frame, time)
        place_pose['place'] = point_to_pose_stamped(place_xyz, place_rpy, frame, time)
        return place_pose

    def transform_dict(self, pose_dict, to_frame):
        """
            transform a dictionary of poses to a certain frame
        """
        for key in pose_dict:
            original_frame = pose_dict[key].header.frame_id
            transform = get_transform(to_frame, original_frame, self.tfBuffer)

            if transform is None:
                rospy.logwarn("[%s] Cannot transform pose, failed to lookup transform from %s to %s!", self.node_name, original_frame, to_frame)
                return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

            pose_dict[key] = tf2_geometry_msgs.do_transform_pose(pose_dict[key], transform)
        return pose_dict


    def robot_height(self):
        """
            calculate height of the robot_base_frame with respect to the planning frame
        """
        transform_stamped = self.tfBuffer.lookup_transform(self.planning_frame, self.robot_base_frame, rospy.Time.now())
        return transform_stamped.transform.translation.z

    def transform_pose(self):

        if not wait_for_variable(self.object_features, 3):
            rospy.logwarn("[%s] Cannot transform pose, since object_features still empty!", self.node_name)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        # transform cage location to world frame
        object_pose = self.object_features.cage_location
        original_frame = object_pose.header.frame_id
        transform = get_transform(self.planning_frame, original_frame, self.tfBuffer)

        if transform is None:
            rospy.logwarn("[%s] Cannot transform pose, failed to lookup transform from %s to %s!", self.node_name, original_frame, self.planning_frame)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        object_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')

        # override the grasp height using the defined peduncle and surface height
        grasp_height = self.peduncle_height - self.surface_height + self.robot_height()
        object_pose.pose.position = list_to_position((object_position[0], object_position[1], grasp_height))

        # update orientation to take into account wrist limits
        object_angle = object_orientation[2]
        base_angle = np.arctan2(object_position[1], object_position[0])

        if self.wrist_upper_limit > (object_angle + base_angle) > self.wrist_lower_limit:
            object_angle = object_angle + np.pi

        object_pose.pose.orientation = list_to_orientation([0, 0, object_angle])  #


        action_pose = {}
        for key in self.pose_transform:
            added_pose = PoseStamped()
            added_pose.header.frame_id = object_pose.header.frame_id
            added_pose.header.stamp = object_pose.header.stamp
            added_pose.pose = add_pose(self.pose_transform[key], object_pose.pose)

            if added_pose is None:
                rospy.logwarn("[%s] Failed to add pose stamped", self.node_name)
                return FlexGraspErrorCodes.FAILURE
            else:
                action_pose[key] = added_pose

        # generate place poses and append to
        place_poses = self.generate_place_pose()
        place_poses = self.transform_dict(place_poses, self.planning_frame)
        action_pose.update(place_poses)

        for key in action_pose:
            rospy.loginfo("[%s] %s height %s [m]", self.node_name, key, action_pose[key].pose.position.z)

        # compensate
        for key in action_pose:
            x = action_pose[key].pose.position.x
            y = action_pose[key].pose.position.y
            r = (x**2 + y**2)**0.5
            delta_height = np.tan(self.sag_angle)*r
            rospy.loginfo("[TRANSFORM POSE] %s add height %s [m] due to radius %s [m]", key, delta_height, r)
            action_pose[key].pose.position.z += delta_height

        self.action_pose = action_pose
        self.pub_all_poses()
        return FlexGraspErrorCodes.SUCCESS

    def pub_all_poses(self):
        for key in self.pose_pub:
            self.pose_pub[key].publish(self.action_pose[key])

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        if self.command == "transform":
            rospy.logdebug("[%s] executing transform command", self.node_name)
            result = self.transform_pose()

        elif self.command == "e_init":
            result = FlexGraspErrorCodes.SUCCESS

        # publish success
        if result is not None:
            msg.val = result
            flex_grasp_error_log(result, self.node_name)
            self.pub_e_out.publish(msg)
            self.command = None


def main():
    try:
        transform_pose = TransformPose()
        while not rospy.core.is_shutdown():
            transform_pose.take_action()
            transform_pose.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
