#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:49:29 2020

@author: jelle
"""

import sys
sys.path.insert(0, '/home/taeke/catkin_ws/src/flexcraft_jelle/flex_grasp/src/')

import numpy as np

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from flex_grasp.msg import Truss

from func.conversions import pose_to_lists
from func.utils import add_lists
from moveit_commander.conversions import list_to_pose
from math import pi
from func.conversions import list_to_orientation

from func.ros_utils import wait_for_success

import tf2_ros
import tf2_geometry_msgs

class PoseTransform(object):
    """PoseTransform"""
    def __init__(self):
        self.event = None
        self.object_features = None

        self.debug_mode = rospy.get_param("pose_transform/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[POINT] Luanching object detection node in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("point",
                        anonymous=True, log_level=log_level)

        # Initialize Subscribers
        rospy.Subscriber("object_features", Truss, self.object_features_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Initialize Publishers
        self.pub_move_robot_command = rospy.Publisher("move_robot/e_in",
                          String, queue_size=10, latch=False)
                          
        self.pub_tomato_point = rospy.Publisher('tomato_point',
                                    PointStamped, queue_size=5, latch=True)   
                                
        self.pub_tomato_point_trans = rospy.Publisher('tomato_point_trans',
                            PointStamped, queue_size=5, latch=True) 

        self.pub_robot_pose = rospy.Publisher('robot_pose',
                                PoseStamped, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.planning_frame = rospy.get_param('planning_frame')
        self.robot_base_frame =  rospy.get_param('robot_base_frame')
        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        self.use_sdh = rospy.get_param('use_sdh')

        if self.use_iiwa:
            self.point_position_transform = [0, 0, 0.05] # [m]
            self.orientation_transform = [0, 0, -pi/2]
        if self.use_interbotix:
            self.point_position_transform = [0.0, 0.0, 0.15] # [m] 0.08
            self.orientation_transform = [-pi, pi/2, 0]


        # Listen
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


        #wait until transform is obtained
        self.trans = None


    def object_features_cb(self, msg):
        if self.object_features is None:
            self.object_features = msg
            rospy.logdebug("[POINT] Received new object feature message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[POINT] Received new pose transform event message %s", self.event)

            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)
            # self.pub_e_out.publish(msg_e)
            # msg_e = String()
            # msg_e.data = "e_wait"

    def get_trans(self, from_frame, to_frame):
        try:
            trans_time = rospy.Time(0) # rospy.Time.now() # rospy.get_time() # 
            
            rospy.logdebug("Transform from: %s", from_frame)
            rospy.logdebug("Transform to: %s", to_frame)
            
            trans = self.tfBuffer.lookup_transform(to_frame, from_frame, trans_time)
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
            # continue

    def transform_pose(self):
        if self.object_features is None:
            rospy.logwarn("[POSE TRANSFORM] Cannot transform pose, since object_features still empty!")
            return False
        else:
            success = True            
            
            for tomato in self.object_features.tomatoes:
                
                tomato_point = PointStamped()
                tomato_point.header = tomato.header
                tomato_point.point = tomato.position
                
                trans = self.get_trans(tomato.header.frame_id, self.robot_base_frame)
                tomato_point_trans = tf2_geometry_msgs.do_transform_point(tomato_point, trans)
                
                # tomato_point_trans = tomato_pose
                # tomato_point_trans.point.z = 0.05  
                ai = 0.0
                aj = 0.0
                ak = np.arctan(tomato_point_trans.point.y/tomato_point_trans.point.x) + np.pi
                orientation = list_to_orientation([ai, aj, ak])
            
                
                tomato_pose = Pose()
                tomato_pose.position = tomato_point_trans.point
                tomato_pose.orientation = orientation
                
                tomato_pose_stamped = PoseStamped()
                
                tomato_pose_stamped.header = tomato_point_trans.header 
                tomato_pose_stamped.pose = tomato_pose
                
                trans = self.get_trans(tomato_pose_stamped.header.frame_id, self.planning_frame)    
                tomato_pose_stamped_trans = tf2_geometry_msgs.do_transform_pose(tomato_pose_stamped, trans)                
                
                pre_grasp_pose_stamped = self.object_pose_to_grasp_pose(tomato_pose_stamped_trans, self.point_position_transform)


                self.pub_tomato_point.publish(tomato_point)
                self.pub_tomato_point_trans.publish(tomato_point_trans)
                self.pub_robot_pose.publish(pre_grasp_pose_stamped)
                self.pub_move_robot_command.publish("move_manipulator")
    
                # get response
                success = wait_for_success("move_robot/e_out", 5)
                
                if success:
                    rospy.loginfo("[POINT] Succesfully pointed at tomato")
                else:
                    rospy.logwarn("[POINT] Failed to point at tomtao")

                self.pub_move_robot_command.publish("home")
                success = wait_for_success("/px150/move_robot/e_out", 5)

            # reset
            self.object_features = None
            return success

    def object_pose_to_grasp_pose(self, object_pose, position_transform):

        grasp_pose = PoseStamped()
        grasp_pose.header = object_pose.header

        # position
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        grasp_position = add_lists(object_position, position_transform)
        grasp_orientation = add_lists(object_orientation, self.orientation_transform)

        grasp_pose.pose = list_to_pose(grasp_position + grasp_orientation)

        return grasp_pose

    def take_action(self):

        success = None
        msg = String()

        if self.event == "e_start":
            success = self.transform_pose()

        elif self.event == "e_init":
            rospy.logdebug("[POINT] executing e_init command")
            success = True

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.event = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Point failed to execute: %s", self.event)
                self.event = None

            self.pub_e_out.publish(msg)

def main():
    try:
        pose_transform = PoseTransform()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            pose_transform.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
