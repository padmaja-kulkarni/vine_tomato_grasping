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
from detect_crop.ProcessImage import ProcessImage


# import pathlib
import os # os.sep

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):
        
        self.event = None
        self.image = None
        self.bridge = CvBridge()
        
        pathCurrent = os.path.dirname(__file__) # path to THIS file
        self.pwdProcess = os.path.join(pathCurrent, '..', '..', 'results')
        
        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)
        
        # Subscribe
        
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("/realsense_plugin/camera/color/image_raw", Image, self.image_cb)
        
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
                self.image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                rospy.logdebug("Image dimensions: %s", self.image.shape)
            except CvBridgeError as e:
                print(e)
        
        
    def detect_object(self):
        if self.event == "e_start":
            
            # if False: #self.image is not None:
            pwd = os.path.dirname(__file__)
            rospy.logdebug("====Initializing image processing object====")
            
            image = ProcessImage(self.image, tomatoName = 'gazebo_tomato', 
                                 pwdProcess = pwd, 
                                 saveIntermediate = True)
            
            rospy.logdebug("====Processing image====")
            image.process_image()
            rospy.logdebug("====Done====")
            
            
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
        
