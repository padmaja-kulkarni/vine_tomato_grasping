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
from sensor_msgs.msg import CameraInfo

# custom func
from detect_crop.ProcessImage import ProcessImage

import pyrealsense2 as rs

# import pathlib
import os # os.sep

def get_intrinsics(camera_info):
    
    # init object
    intrin = rs.intrinsics()
    
    # dimensions
    intrin.width = camera_info.width
    intrin.height = camera_info.height
    
    # principal point coordinates
    intrin.ppx = camera_info.K[2]
    intrin.ppy = camera_info.K[5]

    # focal point
    intrin.fx = camera_info.K[0]
    intrin.fy = camera_info.K[4]

    return intrin

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):
        
        self.event = None
        
        self.color_image = None
        self.depth_image = None
        self.depth_info = None
        self.color_info = None
        
        
        self.bridge = CvBridge()
        
        pathCurrent = os.path.dirname(__file__) # path to THIS file
        self.pwdProcess = os.path.join(pathCurrent, '..', '..', 'results')
        
        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)
        
        # Subscribe
        
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("/realsense_plugin/camera/color/image_raw", Image, self.color_image_cb)
        rospy.Subscriber("/realsense_plugin/camera/depth/image_raw", Image, self.depth_image_cb)
        rospy.Subscriber("/realsense_plugin/camera/color/camera_info", CameraInfo, self.color_info_cb)
        rospy.Subscriber("/realsense_plugin/camera/depth/camera_info", CameraInfo, self.depth_info_cb)
        
        # Publish
        
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)
        
        self.pub_pose = rospy.Publisher("~objectPose",
                                        PoseStamped, queue_size=5, latch=True)
        
    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("Received new move robot event message: %s", self.event)
     
    def color_image_cb(self, msg):
        if (self.color_image is None) and (self.event == "e_start"):
            rospy.logdebug("Received color image message")
            rospy.logdebug("Height: %s", msg.height)
            rospy.logdebug("Width: %s", msg.width)
            rospy.logdebug("header: \n %s", msg.header)
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                print(e)
        
    def depth_image_cb(self, msg):
        if (self.depth_image is None) and (self.event == "e_start"):
            rospy.logdebug("Received depth image message")
            rospy.logdebug("Height: %s", msg.height)
            rospy.logdebug("Width: %s", msg.width)
            rospy.logdebug("header: \n %s", msg.header)
            try:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            except CvBridgeError as e:
                print(e)
            
            
            
    def color_info_cb(self, msg):
        if (self.color_info is None) and (self.event == "e_start"):
            rospy.logdebug("Received color info message")
            self.color_info = msg
            
    def depth_info_cb(self, msg):
        if (self.depth_info is None) and (self.event == "e_start"):
            rospy.logdebug("Received depth info message")
            self.depth_info = msg
        
    def detect_object(self):
        if self.event == "e_start":
            
            if (self.color_image is not None) and (self.depth_image is not None) and (self.depth_info is not None) and (self.color_info is not None):
                pwd = os.path.dirname(__file__)
                rospy.logdebug("====Initializing image processing object====")
                
                image = ProcessImage(self.color_image, tomatoName = 'gazebo_tomato', 
                                     pwdProcess = pwd, 
                                     saveIntermediate = False)
                
                rospy.logdebug("====Processing image====")
                image.process_image()
                rospy.logdebug("====Done====")
                grasp_pixel = image.graspO
                rospy.logdebug("Obtained location in pixel frame: %s ", grasp_pixel)
                
                # Deproject
                index = (grasp_pixel[1], grasp_pixel[0])
                depth = self.depth_image[index]
                rospy.logdebug("Corresponding depth: %s", self.depth_image[index])
                # https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
                
                
                intrin = get_intrinsics(self.depth_info) # self.depth_info.get_intrinsics()
                pixel = [float(grasp_pixel[0]), float(grasp_pixel[1])]
                depth = float(depth)
                
                rospy.logdebug("intrin: %s, of type:, %s ", intrin, type(intrin))
                rospy.logdebug("pixel: %s, of type :, %s ", pixel, type(pixel))
                rospy.logdebug("depth: %s, of type :, %s ", depth, type(depth))
                
                
                
                depth_point = rs.rs2_deproject_pixel_to_point(intrin, pixel, depth)
                rospy.logdebug("Depth point: %s [mm]", depth_point)
            
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
        
