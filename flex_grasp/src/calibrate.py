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
        
        self.frame = "px150/base_link"
    
        self.pub_move_robot = rospy.Publisher("move_robot/e_in",
                                  String, queue_size=10, latch=True)
        
        self.pub_move_robot = rospy.Publisher("pre_grasp_pose",
                                  PoseStamped, queue_size=10, latch=True)
        
        self.pub_pose_array = rospy.Publisher("pose_array",
                                PoseArray, queue_size=5, latch=True)
        
        
    def init_poses(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame
        pose_array.header.stamp = rospy.Time.now()
        
        

        
        x_amplitude = 0.1
        y_amplitude = 0.1
        z_amplitude = 0.05
        
        x_min = 0.15
        y_min = -y_amplitude
        z_min = 0.05

        
        intervals = 3
        x_vec = np.linspace(x_min, x_min + 2*x_amplitude, intervals)
        y_vec = np.linspace(y_min, y_min + 2*y_amplitude, intervals)
        z_vec = np.linspace(z_min, z_min + 2*z_amplitude, intervals)
        
        ai = 0
        aj = 0
        ak = 0
        
        poses = []
        
        for x in x_vec:
            for y in y_vec:
                for z in z_vec:
                    pose = Pose()
                    pose.position = list_to_position([x, y, z])
                    pose.orientation = list_to_orientation([ai, aj, ak])
                    
                    poses.append(pose)
                    
        pose_array.poses = poses
        self.pose_array = pose_array


def main():
    try:
        calibrate = Calibrate()

        
        calibrate.init_poses()
        calibrate.pub_pose_array.publish(calibrate.pose_array)
        
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
