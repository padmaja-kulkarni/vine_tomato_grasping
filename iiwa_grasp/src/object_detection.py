#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: jelle
"""

import rospy
from cv_bridge import CvBridge, CvBridgeError

# msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# custom func
from func.image_processing import segmentation_otsu
from func.image_processing import save_fig


# import pathlib
import os # os.sep

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):
        
        self.event = None
        self.image = None
        self.bridge = CvBridge()
        
        pathCurrent = "/home/taeke/catkin_ws/src/flexcraft_jelle/iiwa_grasp/src"#
        self.pwdProcess = os.path.join(pathCurrent)
        
        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)
        
        # Subscribe
        
        self.e_in_cb_lambda = lambda msg: self.e_in_cb(msg)
        rospy.Subscriber("~e_in",
                         String, self.e_in_cb_lambda)
        
        self.image_cb_lambda = lambda msg: self.image_cb(msg)
        rospy.Subscriber("/realsense_plugin/camera/color/image_raw",
                         Image, self.image_cb_lambda)
        
        # Publish
        
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)
        
        self.pub_pose = rospy.Publisher("~objectPose",
                                        PoseStamped, queue_size=5, latch=True)
        
    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("Received new move robot event message: %s", self.event)
     
    def image_cb(self, msg):
        if self.image is None:
            rospy.logdebug("Received new image message")
            try:
                self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
        
        
    def detect_object(self):
        if self.event == "e_start":
            
            if self.image is not None:
                
                rospy.logdebug("===PROCESSING IMAGE====")
                background, tomato, peduncle = segmentation_otsu(self.image, 255)
                
                rospy.logdebug("===SAVING IMAGE====")
                save_fig(background, self.pwdProcess, '02_a', figureTitle = "Background")
                save_fig(tomato, self.pwdProcess, '02_b', figureTitle = "Tomato")
                save_fig(peduncle, self.pwdProcess, '02_c', figureTitle = "Peduncle")
                
                
            
            
            self.event = None
            msg_pose = PoseStamped()
            msg_pose.header.frame_id = rospy.get_param('planning_frame')
            msg_pose.header.stamp = rospy.Time.now()
            
            msg_pose.pose.orientation.x = -0.310
            msg_pose.pose.orientation.y = 0.000
            msg_pose.pose.orientation.z = 0.001
            msg_pose.pose.orientation.w = 0.951
            msg_pose.pose.position.x = -0.014
            msg_pose.pose.position.y = 0.262
            msg_pose.pose.position.z = 1.127
            
            msg_e = String()
            msg_e.data = "e_success"
            
            self.event = None
            self.pub_pose.publish(msg_pose)
            self.pub_e_out.publish(msg_e)
            
            
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
        
