#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 14:28:49 2020

@author: taeke
"""

import rospy
import numpy as np

# messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

import tf2_ros # for error messages
import tf2_geometry_msgs

from func.conversions import list_to_position, list_to_orientation


class Calibrate(object):
    """Calibrate"""
    def __init__(self):    

        self.debug_mode = rospy.get_param("calibrate/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[CALIBRATE] Luanching move robot in debug mode")
        else:
            log_level = rospy.INFO
        
        rospy.init_node("move_robot",
                anonymous=True,
                log_level=log_level)
        
        # Listen
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        
        self.frame = "px150/base_link"
    
        self.pub_move_robot_command = rospy.Publisher("move_robot/e_in",
                                  String, queue_size=10, latch=True)
        
        self.pub_move_robot_pose = rospy.Publisher("pre_grasp_pose",
                                  PoseStamped, queue_size=10, latch=True)
        
        self.pub_pose_array = rospy.Publisher("pose_array",
                                PoseArray, queue_size=5, latch=True)
        
        
    def init_poses(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame
        pose_array.header.stamp = rospy.Time.now()
        
        

        
        x_amplitude = 0.05
        y_amplitude = 0.08
        z_amplitude = 0.05
        
        x_min = 0.25
        y_min = -y_amplitude
        z_min = 0.08 # 0.05

        
        intervals = 3
        x_vec = np.linspace(x_min, x_min + 2*x_amplitude, intervals)
        y_vec = np.linspace(y_min, y_min + 2*y_amplitude, intervals)
        z_vec = np.linspace(z_min, z_min + 2*z_amplitude, intervals)
        
        ai_amplitude = 20.0/180*np.pi
        aj_amplitude = 20.0/180*np.pi
        
        ai_vec = np.linspace(-ai_amplitude, ai_amplitude, intervals)
        aj_vec = np.linspace(-aj_amplitude, aj_amplitude, intervals)
        
        poses = []
        
        for x in x_vec:
            for y in y_vec:
                for z in z_vec:
                    for aj in aj_vec:
                        for ai in ai_vec:
                            pose = Pose()
                            pose.position = list_to_position([x, y, z])
                            
                            ak = np.arctan(y/x)
                            pose.orientation = list_to_orientation([ai, aj, ak])
                            
                            poses.append(pose)
                    
        pose_array.poses = poses
        self.pose_array = pose_array
        
    def go_to_poses(self):
        
        try:
            trans = self.tfBuffer.lookup_transform('world',self.pose_array.header.frame_id,  rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
        for pose in self.pose_array.poses:
            
            pose_stamped = PoseStamped()
            pose_stamped.header = self.pose_array.header
            pose_stamped.pose = pose
            
            
            pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
            
            
            
            self.pub_move_robot_pose.publish(pose_trans)
            self.pub_move_robot_command.publish("move")
            
            # get response
            success = wait_for_success("move_robot/e_out", 10)
        
            if success:
                pass

def wait_for_success(topic, timeout):


    start_time = rospy.get_time()
    curr_time = rospy.get_time()

    # rospy.logdebug("==WAITING FOR SUCCESS==")
    # rospy.logdebug("start time: %s", start_time)
    # rospy.logdebug("current time: %s", curr_time)
    while (curr_time - start_time < timeout): # and not rospy.is_shutdown():
        # rospy.logdebug("current time: %s", curr_time)
        try:
            message = rospy.wait_for_message(topic, String, timeout)
            if message.data == "e_success":
                rospy.logdebug("[CALIBRATE] Command succeeded: received %s on topic %s", message.data, topic)
                return True
            elif message.data == "":
                pass
            else:
                rospy.logwarn("[CALIBRATE] Command failed: node returned %s on topic %s", message.data, topic)
                return False
        except:
            rospy.logwarn("[CALIBRATE] Command failed: timeout exceeded while waiting for message on topic %s", topic)
            return False

        rospy.sleep(0.2)
        curr_time = rospy.get_time()

    rospy.logwarn("[CALIBRATE] Command failed: node did not return success within timeout on topic %s", topic)
    return False


def main():
    try:
        
        calibrate = Calibrate()
        rate = rospy.Rate(10)
        
        calibrate.init_poses()
        calibrate.pub_pose_array.publish(calibrate.pose_array)
        
        rospy.sleep(10)
        
        calibrate.go_to_poses()
        
        while not rospy.core.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
