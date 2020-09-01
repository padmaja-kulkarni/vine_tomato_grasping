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
from func.utils import add_pose_stamped
from func.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position
from func.flex_grasp_error import flex_grasp_error_log

import tf2_ros
import tf2_geometry_msgs

class TransformPose(object):
    
    node_name = 'TRANSFORM POSE'   
    frequency = 10 # [hz]
    
    def __init__(self):
        
        self.debug_mode = rospy.get_param("transform_pose/debug")        
        
        # initialize node        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[%s] Launching pick place node in debug mode", self.node_name)
        else:
            log_level = rospy.INFO
        
        rospy.init_node("transform_pose", anonymous=True, log_level=log_level)
        self.rate = rospy.Rate(self.frequency)   
        
        # params
        self.surface_height = 0.018
        self.peduncle_height = 0.075 # [m]
        grasp_xyz = [0, 0, 0.065] # [m]
        pre_grasp_xyz = [0, 0, 0.11] # [m]
        grasp_rpy = [-np.pi, np.pi/2, 0]        
        
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
        keys = ['pre_grasp', 'grasp', 'pre_place','place']
        pose = dict.fromkeys(keys)        
        
        # Initialize Publishers
        values = [x + '_pose' for x in keys]
        pose_topic = dict(zip(keys, values))
        
        pose_pub = {}
        for key in pose_topic:
            pose_pub[key] = rospy.Publisher(pose_topic[key], PoseStamped, queue_size=5, latch=True)
        
        
        random.seed(0)
        frame = self.robot_base_frame
        time = rospy.Time.now()        

        pose_transform = {}
        pose_transform['pre_grasp'] = point_to_pose_stamped(pre_grasp_xyz, grasp_rpy, frame, time)
        pose_transform['grasp'] = point_to_pose_stamped(grasp_xyz, grasp_rpy, frame, time)

        
        self.pre_grasp_xyz = pre_grasp_xyz
        self.grasp_xyz = grasp_xyz
        self.grasp_rpy = grasp_rpy

        self.pose_pub = pose_pub
        self.pose = pose
        self.pose_transform = pose_transform

        # Tranform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

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
        rospy.logdebug("[PICK PLACE] Received new object feature message")  

    def peduncle_height_cb(self, msg):
        self.peduncle_height = msg.data
        rospy.logdebug("[PICK PLACE] Received new peduncle height: %s", self.peduncle_height)  
        if self.object_features is not None:
            self.transform_pose()

    def generate_place_pose(self, pre_place_height, place_height):
        
        x = 0.0
        while x <= self.x_min:
            theta = np.deg2rad(random.randrange(-90, 90, 1)) # [rad]
            r = self.r_min + random.random()*(self.r_max - self.r_min) # [m]    
            
            x = r * np.cos(theta)
            y = r * np.sin(theta)
        
        orientation = np.deg2rad(random.randrange(90, 270, 1)) + theta # [rad]        
        
        place_rpy = [self.grasp_rpy[0], self.grasp_rpy[1], orientation]
        pre_place_xyz = [x, y, pre_place_height]      
        place_xyz = [x, y, place_height]        

        frame = self.robot_base_frame# robot_base_frame
        time = rospy.Time.now()
        
        place_pose = {}
        place_pose['pre_place'] = point_to_pose_stamped(pre_place_xyz, place_rpy, frame, time)
        place_pose['place'] = point_to_pose_stamped(place_xyz, place_rpy, frame, time)        
        return place_pose

    def transform_dict(self, pose_dict, to_frame):
        # transfrom add_pose to planning frame
        for key in pose_dict:
            original_frame = pose_dict[key].header.frame_id
            transform = get_transform(to_frame, original_frame, self.tfBuffer)            
                
            if transform is None:
                rospy.logwarn("[PICK PLACE] Cannot transform pose, failed to lookup transform!!")
                return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED
    
            pose_dict[key] = tf2_geometry_msgs.do_transform_pose(pose_dict[key], transform)
        return pose_dict

    def transform_pose(self, angle_offset = 0):
        if not wait_for_variable(3, self.object_features):
            rospy.logwarn("[PICK PLACE] Cannot transform pose, since object_features still empty!")
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED
            
        # transform cage location to world frame
        object_pose = self.object_features.cage_location
        original_frame = object_pose.header.frame_id
        transform = get_transform(self.planning_frame, original_frame, self.tfBuffer)            
            
        if transform is None:
            rospy.logwarn("[PICK PLACE] Cannot transform pose, failed to lookup transform!!")
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        object_pose = tf2_geometry_msgs.do_transform_pose(object_pose, transform)
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        
        grasp_height = self.peduncle_height - self.surface_height 
        object_pose.pose.position = list_to_position((object_position[0], object_position[1], grasp_height))
        
        object_angle = -object_orientation[2] + angle_offset
        base_angle = np.arctan2(object_position[1], object_position[0])
        
        if object_angle > (-base_angle + 0.5*np.pi) or object_angle < (-base_angle - 0.5*np.pi):
            pass
        else:
            object_angle = object_angle + np.pi

        rospy.logdebug("[%s] object angle: %s", self.node_name, np.rad2deg(object_angle))
        rospy.logdebug("[%s] base angle: %s", self.node_name, np.rad2deg(base_angle))

        object_pose.pose.orientation = list_to_orientation((object_angle, 0, 0)) # 

        # transfrom add_pose to planning frame
        pose_transform = self.transform_dict(self.pose_transform, self.planning_frame)
        
        pose = {}
        for key in pose_transform:
            added_pose = add_pose_stamped(pose_transform[key], object_pose) 

            if added_pose is None:
                rospy.logwarn('[PICK PLACE] Failed to add pose stamed: they are defiend with respect to different frames!')
                return FlexGraspErrorCodes.FAILURE
            else:
                pose[key] = added_pose
  
  
        place_poses = self.generate_place_pose(pose['pre_grasp'].pose.position.z, pose['grasp'].pose.position.z)  
        place_poses = self.transform_dict(place_poses, self.planning_frame)
        for key in place_poses:
                pose[key] = place_poses[key]          
            

        self.pose = pose

        
        self.pub_all_poses()
        return FlexGraspErrorCodes.SUCCESS

    def pub_all_poses(self):
        for key in self.pose_pub:
            self.pose_pub[key].publish(self.pose[key])
  

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None
        
        if self.command == "transform":
            rospy.logdebug("[PICK PLACE] executing transform command")
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
