#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: jelle
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):
        
        self.event = None
        
        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)
        
        self.e_in_cb_lambda = lambda msg: self.e_in_cb(msg, self)
        rospy.Subscriber("~e_in",
                         String, self.e_in_cb_lambda)
        
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)
        
        self.pub_pose = rospy.Publisher("~objectPose",
                                        PoseStamped, queue_size=5, latch=True)
        
    def e_in_cb(msg, self):
        if self.event is None:
            self.event = "e_start"
            rospy.logdebug("Received new move robot event message")
            
    def detect_object(self):
        if self.event == "e_start":
            self.event = None
            msg_pose = PoseStamped
            msg_pose.header.frame_id = rospy.get_param('planning_frame')
            msg_pose.header.stamp = rospy.Time.now()
            
            msg_pose.pose.orientation.x = -0.310
            msg_pose.pose.orientation.y = 0.000
            msg_pose.pose.orientation.z = 0.001
            msg_pose.pose.orientation.w = 0.951
            msg_pose.pose.position.x = -0.014
            msg_pose.pose.position.y = 0.262
            msg_pose.pose.position.z = 1.127
            
            msg_e = String
            msg_e.data = "e_success"
            
            self.pub_pose.publish(msg_pose)
            self.pub_e_out(msg_e)
            
            
def main():
    try:
        OD = ObjectDetection()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            OD.detect_object()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == '__main__':
    main()
        
