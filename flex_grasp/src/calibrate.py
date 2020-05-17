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

from easy_handeye.handeye_client import HandeyeClient


# custom functions
from func.ros_utils import wait_for_success
from func.conversions import list_to_position, list_to_orientation

class Calibration(object):
    """Calibration"""
    def __init__(self):    

        self.debug_mode = rospy.get_param("/px150/calibration_eye_on_base/calibrate/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
        else:
            log_level = rospy.INFO
        
        rospy.init_node("move_robot",
                anonymous=True,
                log_level=log_level)
        
        if self.debug_mode:
            rospy.loginfo("[CALIBRATE] Luanching calibrate in debug mode")
        

        rospy.sleep(5)
        
        rospy.logdebug("[CALIBRATE] initializing hand eye client")
        self.client = HandeyeClient()
        
        
        # Listen
        rospy.logdebug("[CALIBRATE] initializing tf2_ros buffer")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        
        self.calibration_frame = "px150/base_link"
        self.planning_frame = "world"
        self.pose_array = None
        
        self.event = None
        
        
        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        
        # we need to be sucsribed to the aruco tracker for it to publish the tf
        rospy.Subscriber("/px150/aruco_tracker/pose", PoseStamped, self.aruco_tracker_cb)
    
        self.pub_e_out = rospy.Publisher("~e_out",
                                     String, queue_size=10, latch=True)
    
        self.pub_move_robot_command = rospy.Publisher("/px150/move_robot/e_in",
                                  String, queue_size=10, latch=True)
        
        self.pub_move_robot_pose = rospy.Publisher("/px150/pre_grasp_pose",
                                  PoseStamped, queue_size=10, latch=True)
        
        self.pub_pose_array = rospy.Publisher("/px150/pose_array",
                                PoseArray, queue_size=5, latch=True)
        
    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[CALIBTRATION] Received object detection event message: %s", self.event)
        
            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)    
        
        
    def aruco_tracker_cb(self, msg):
        pass
        
    def init_poses(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.calibration_frame
        pose_array.header.stamp = rospy.Time.now()
        
        x_amplitude = 0.05
        y_amplitude = 0.08
        z_amplitude = 0.05
        
        x_min = 0.25
        y_min = -y_amplitude
        z_min = 0.08 # 0.05

        
        pos_intervals = 1
        if pos_intervals == 1:
            x_vec = [x_min + x_amplitude]
            y_vec = [y_min + y_amplitude]
            z_vec = [z_min + z_amplitude]
        else:
            x_vec = np.linspace(x_min, x_min + 2*x_amplitude, pos_intervals)
            y_vec = np.linspace(y_min, y_min + 2*y_amplitude, 2)
            z_vec = np.linspace(z_min, z_min + 2*z_amplitude, pos_intervals)
        
        ai_amplitude = 30.0/180*np.pi
        aj_amplitude = 30.0/180*np.pi
        
        rot_intervals = 3
        ai_vec = np.linspace(-ai_amplitude, ai_amplitude, rot_intervals)
        aj_vec = np.linspace(-aj_amplitude, aj_amplitude, rot_intervals)
        
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
        
        self.pub_pose_array.publish(pose_array)
        self.pose_array = pose_array
        return True
        
        
    def calibrate(self):
        
        # does pose array contain something?
        if self.pose_array is None:
            rospy.logwarn("[CALIBRATE] pose_array is still empty")
            return False
        
        try:
            trans = self.tfBuffer.lookup_transform(self.planning_frame,self.pose_array.header.frame_id,  rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[CALIBRATE] failed to get transform from %s to %s", self.pose_array.header.frame_id, self.planning_frame)
            return False

        
        for pose in self.pose_array.poses:
            if rospy.is_shutdown():
                return False
            
            pose_stamped = PoseStamped()
            pose_stamped.header = self.pose_array.header
            pose_stamped.pose = pose
            
            
            pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
            
            
            
            self.pub_move_robot_pose.publish(pose_trans)
            self.pub_move_robot_command.publish("move")
            
            # get response
            success = wait_for_success("/px150/move_robot/e_out", 5)
            attempts = 1
            
            if success:
                # wait a small amount of time for vibrations to stop
                rospy.sleep(0.5)
                
                for attempt in range(0,attempts):
                    
                    try:
                        self.client.take_sample()
                        break
                    except:
                        rospy.logwarn("[CALIBRATE] Failed to take sample, marker might not be visible. Attempts remaining: %s", attempts - attempt - 1)


        result = self.client.compute_calibration()
        self.result = result
        
        if result.valid:
            rospy.loginfo("Found valid transfrom")
            self.broadcast(result)
            success = True
        else:
            rospy.logwarn("Computed calibration is invalid")
            success = False
        
        self.client.save()
        
        return success


    def broadcast(self, result):
        rospy.loginfo("Broadcasting result")
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        static_transformStamped = result.calibration.transform
        broadcaster.sendTransform(static_transformStamped)

    def take_action(self):
        success = None
        msg = String()

        if (self.event == "e_init"):
            success = self.init_poses()

        elif (self.event == "e_start"):
            success = self.calibrate()
            

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.event = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Calibration failed to execute command: %s", self.event)
                self.event = None

            self.pub_e_out.publish(msg)


def main():
    try:
        calibration = Calibration()
        rate = rospy.Rate(10)
        
        while not rospy.core.is_shutdown():
            calibration.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
