#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: jelle
"""

import rospy
import tf

import math
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

# msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from flex_grasp.msg import Tomato
from flex_grasp.msg import Truss
from flex_grasp.msg import Peduncle

# custom func
from detect_crop.ProcessImage import ProcessImage

import pyrealsense2 as rs

# import pathlib
import os # os.sep

def camera_info2intrinsics(camera_info):

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
        self.trans = None

        self.bridge = CvBridge()

        pathCurrent = os.path.dirname(__file__) # path to THIS file
        self.pwdProcess = os.path.join(pathCurrent, '..', '..', 'results')


        self.debug_mode = rospy.get_param("object_detection/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[OBJECT DETECTION] Luanching object detection node in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("object_detection", anonymous=True, log_level=log_level)

        # Publish
        latch = True
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=latch)

        self.pub_object_features = rospy.Publisher("object_features",
                                        Truss, queue_size=5, latch=True)

        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        if not self.debug_mode:
            rospy.Subscriber("camera/color/image_raw", Image, self.color_image_cb)
            rospy.Subscriber("camera/depth/image_rect_raw", Image, self.depth_image_cb)
            rospy.Subscriber("camera/color/camera_info", CameraInfo, self.color_info_cb)
            rospy.Subscriber("camera/color/camera_info", CameraInfo, self.depth_info_cb)

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[OBJECT DETECTION] Received object detection event message: %s", self.event)

            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)

    def color_image_cb(self, msg):
        if (self.color_image is None) and (self.event == "e_start"):
            rospy.logdebug("[OBJECT DETECTION] Received color image message")
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                print(e)

    def depth_image_cb(self, msg):
        if (self.depth_image is None) and (self.event == "e_start"):
            rospy.logdebug("[OBJECT DETECTION] Received depth image message")
            try:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            except CvBridgeError as e:
                print(e)

    def color_info_cb(self, msg):
        if (self.color_info is None) and (self.event == "e_start"):
            rospy.logdebug("[OBJECT DETECTION] Received color info message")
            self.color_info = msg

    def depth_info_cb(self, msg):
        if (self.depth_info is None) and (self.event == "e_start"):
            rospy.logdebug("[OBJECT DETECTION] Received depth info message")
            self.depth_info = msg

    def received_all_data(self):
        return (self.color_image is not None) and (self.depth_image is not None) and (self.depth_info is not None) and (self.color_info is not None)

    def clear_all_data(self):
        self.color_image = None
        self.depth_image = None
        self.depth_info = None
        self.color_info = None

    def wait_for_data(self, time_out):
        while not self.received_all_data():
            rospy.sleep(0.1)
        return True

    def detect_object(self):


        if self.wait_for_data(5):
            pwd = os.path.dirname(__file__)


            image = ProcessImage(self.color_image, tomatoName = 'gazebo_tomato',
                                 pwdProcess = pwd,
                                 saveIntermediate = False)

            # rospy.logdebug("Image dimensions: %s", image.DIM)

            image.process_image()
            object_feature = image.get_object_features()
            frame = "camera_color_optical_frame"

            #%%##################
            ### Cage location ###
            #####################

            row = object_feature['grasp']['row']
            col = object_feature['grasp']['col']
            angle = -object_feature['grasp']['angle'] # minus since camera frame is upside down...
            rpy = [0, 0, angle]

            intrin = camera_info2intrinsics(self.depth_info)
            xyz = self.deproject(row, col, intrin)
            cage_pose =  point_to_pose_stamped(xyz, rpy, frame)

            #%%#############
            ### tomatoes ###
            ################
            tomatoes = []

            # rospy.logdebug("cols: %s [px]", col)
            for i in range(0, len(object_feature['tomato']['col'])):

                # Load from struct
                col = object_feature['tomato']['col'][i]
                row = object_feature['tomato']['row'][i]
                radius = object_feature['tomato']['radii'][i]

                point = self.deproject(row, col, intrin)

                depth = self.depth_image[(row, col)]
                point1 = rs.rs2_deproject_pixel_to_point(intrin, [0,0], depth)
                point2 = rs.rs2_deproject_pixel_to_point(intrin, [0,radius], depth)
                radius_m = euclidean(point1, point2)

                # tomatoes.append(point_to_tomato(point, radius_m, frame))

            #%%#############
            ### Peduncle ###
            ################
            peduncle = Peduncle()
            peduncle.pose = cage_pose
            peduncle.radius = 0.01
            peduncle.length = 0.15

            self.create_truss(tomatoes, cage_pose, peduncle)

            # reset
            self.clear_all_data()
            return True
        else:
            rospy.logwarn("Did not receive all data")
            return False

    def generate_object(self):

        #%%##################
        ### Cage location ###
        #####################
        table_height = 0.23
        frame = "world"
        object_x = rospy.get_param("object_x")
        object_y = rospy.get_param("object_y")
        angle = rospy.get_param("object_angle")
        xyz = [object_x, object_y, 0.05 + table_height]
        rpy = [3.1415, 0, angle] #3.1415/2.0

        cage_pose =  point_to_pose_stamped(xyz, rpy, frame)

        #%%#############
        ### Peduncle ###
        ################
        L = 0.15
        peduncle = Peduncle()
        peduncle.pose = cage_pose
        peduncle.radius = 0.005
        peduncle.length = L

        #%%#############
        ### tomatoes ###
        ################
        radii = [0.05, 0.05]
        t1x = xyz[0] + (L/2 + radii[0])*math.cos(angle)
        t1y = xyz[1] - (L/2 + radii[0])*math.sin(angle)
        t2x = xyz[0] - (L/2 + radii[1])*math.cos(angle)
        t2y = xyz[1] + (L/2 + radii[1])*math.sin(angle)
        point1 = [t1x, t1y, table_height]
        point2 = [t2x, t2y, table_height]
        points = [point1, point2]

        tomatoes = []
        for point, radius in zip(points, radii):
            # tomatoes.append(point_to_tomato(point, radius, frame))
            pass

        self.create_truss(tomatoes, cage_pose, peduncle)

        # reset
        self.clear_all_data()
        return True

    def create_truss(self, tomatoes, cage_pose, peduncle):
        #%%##########
        ### Truss ###
        #############
        truss = Truss()
        truss.tomatoes = tomatoes
        truss.cage_location = cage_pose
        truss.peduncle = peduncle

        self.pub_object_features.publish(truss)

    def deproject(self, row, col, intrin):
        # Deproject
        index = (row, col)
        depth = self.depth_image[index]
        # rospy.logdebug("Corresponding depth: %s", self.depth_image[index])
        # https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0

        pixel = [float(col), float(row)]
        depth = float(depth)

        point = rs.rs2_deproject_pixel_to_point(intrin, pixel, depth)
        return point

    def take_action(self):
        success = None
        msg = String()

        if (self.event == "e_start"):
            if not self.debug_mode:
                success = self.detect_object()
            if self.debug_mode:
                success = self.generate_object()

        elif (self.event == "e_init"):
            success = True

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.event = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Object detection failed to execute %s", self.event)
                self.event = None

            self.pub_e_out.publish(msg)

def euclidean(v1, v2):
    return sum((p-q)**2 for p, q in zip(v1, v2)) ** .5

def point_to_pose_stamped(xyz, rpy, frame):

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = rospy.Time.now()

    quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    pose_stamped.pose.position.x = xyz[0]
    pose_stamped.pose.position.y = xyz[1]
    pose_stamped.pose.position.z = xyz[2]

    return pose_stamped


def point_to_tomato(point, radius, frame):

    tomato = Tomato()
    tomato.header.frame_id = frame
    tomato.header.stamp = rospy.Time.now()

    tomato.position.x = point[0]
    tomato.position.y = point[1]
    tomato.position.z = point[2] + radius

    tomato.radius = radius
    return tomato



def main():
    try:
        object_detection = ObjectDetection()
        rospy.loginfo("[OBJECT DETECTION] Initialized")
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            object_detection.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
