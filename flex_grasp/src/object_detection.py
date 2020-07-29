#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: jelle
"""

import numpy as np
import rospy
import math
import cv2
import json
from cv_bridge import CvBridge, CvBridgeError

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import struct 

from flex_grasp.msg import Tomato
from flex_grasp.msg import Truss
from flex_grasp.msg import Peduncle
from flex_grasp.msg import ImageProcessingSettings

# custom func
from detect_truss.ProcessImage import ProcessImage
from func.conversions import point_to_pose_stamped, settings_lib_to_msg, settings_msg_to_lib

from func.utils import camera_info2intrinsics

import pyrealsense2 as rs

# import pathlib
import os # os.sep

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):

        self.event = None

        self.color_image = None
        self.depth_image = None
        self.depth_info = None
        self.color_info = None
        self.pcl = None
        self.trans = None
        self.init = None

        self.take_picture = False

        # params
        self.tomato_radius_max = None


        self.color_frame = "camera_color_optical_frame"
        self.depth_frame = "camera_depth_optical_frame"
        self.camera_frame = "camera_color_optical_frame"

        self.camera_sim = rospy.get_param("camera_sim")
        self.debug_mode = rospy.get_param("object_detection/debug")

        self.patch_size = 7

        self.bridge = CvBridge()

        pwd_current = os.path.dirname(__file__) # path to THIS file
        data_set = 'depth_blue'
        self.pwd_results = os.path.join(pwd_current, '..', '..', 'results')
        self.pwd_data = os.path.join(pwd_current, '..', '..', 'detect_truss', 'src', 'data', data_set)

        if not os.path.isdir(self.pwd_data):
            rospy.loginfo("New path, creating a new folder: " + self.pwd_data)
            os.makedirs(self.pwd_data)

        rospy.loginfo("Storing visiual results in: ", self.pwd_results)

        self.process_image = ProcessImage(name = 'ros_tomato',
                     pwd = self.pwd_results,
                     save = False)

        settings = settings_lib_to_msg(self.process_image.get_settings())

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[OBJECT DETECTION] Luanching object detection node in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("object_detection", anonymous=True, log_level=log_level)

        # Publish
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.pub_object_features = rospy.Publisher("object_features",
                                        Truss, queue_size=5, latch=True)

        self.pub_segment_image = rospy.Publisher("segment_image",
                                        Image, queue_size=5, latch=True)

        self.pub_tomato_image = rospy.Publisher("tomato_image",
                                Image, queue_size=5, latch=True)

        self.pub_color_hue = rospy.Publisher("color_hue",
                        Image, queue_size=5, latch=True)
                                
        self.pub_tomato_mask = rospy.Publisher("tomato_mask",
                        Image, queue_size=5, latch=True)
                                
        self.pub_peduncle_pcl = rospy.Publisher("peduncle_pcl",
                                 PointCloud2, queue_size=10, latch=True)
                                 
        self.pub_tomato_pcl = rospy.Publisher("tomato_pcl",
                                 PointCloud2, queue_size=10, latch=True)
                                 
        pub_image_processing_settings = rospy.Publisher("image_processing_settings",
                                 ImageProcessingSettings, queue_size=10, latch=True)

        pub_image_processing_settings.publish(settings)

        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # if not self.debug_mode:i
        rospy.Subscriber("camera/color/image_raw", Image, self.color_image_cb)
        rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, self.depth_image_cb)
        rospy.Subscriber("camera/color/camera_info", CameraInfo, self.color_info_cb)
        rospy.Subscriber("camera/aligned_depth_to_color/camera_info", CameraInfo, self.depth_info_cb)
        rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.point_cloud_cb)        
        
        
        rospy.Subscriber("image_processing_settings", ImageProcessingSettings, self.image_processing_settings_cb)


    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[OBJECT DETECTION] Received object detection event message: %s", self.event)

            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)

    def color_image_cb(self, msg):
        if (self.color_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received color image message")
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                # self.color_frame = "camera_color_optical_frame" # msg.header.frame_id
            except CvBridgeError as e:
                print(e)

    def depth_image_cb(self, msg):
        if (self.depth_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received depth image message")
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")/1000.0
                # self.depth_frame = "camera_depth_optical_frame" # msg.header.frame_id
            except CvBridgeError as e:
                print(e)

    def color_info_cb(self, msg):
        if (self.color_info is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received color info message")
            self.color_info = msg

    def depth_info_cb(self, msg):
        if (self.depth_info is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received depth info message")
            self.depth_info = msg
            
    def point_cloud_cb(self, msg):
        if (self.pcl is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received point_ cloud info message")
            self.pcl = msg
            
    def image_processing_settings_cb(self, msg):
        self.settings = msg
        rospy.logdebug("[PICK PLACE] Received image processing settings")    

    def received_data(self):
        received = {}
        received['color_img'] = self.color_image is not None

        received['depth_img'] = self.depth_image is not None
        received['depth_info'] = self.depth_info is not None
        received['color_info'] = self.color_info is not None
        received['pcl'] = self.pcl is not None
        
        all_data = True
        for key in received:
            all_data = all_data and received[key]
            
        received['all'] = all_data
            
        return received

    def clear_all_data(self):
        self.color_image = None
        self.depth_image = None
        self.depth_info = None
        self.color_info = None
        self.pcl = None

    def wait_for_data(self, timeout):
        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < timeout):
            received_data = self.received_data()
            if received_data['all'] is True:
                self.take_picture = False
                rospy.logdebug("[OBJECT DETECTION] Received all data")
                return True

            rospy.sleep(0.1)

        rospy.logwarn("[OBJECT DETECTION] Did not receive all data %s", received_data)
        return False


    def save_image(self):

        if not self.wait_for_data(5):
            return False
        
        # imaformation about the image which will be stored
        image_info = {}
        image_info['px_per_mm'] = self.compute_px_per_mm()

        # check contents folder
        contents = os.listdir(self.pwd_data)

        # determine starting number
        if not len(contents):
            new_id = 1;
        else:
            contents.sort()
            file_name = contents[-1]
            file_id = file_name[:3]
            new_id = int(file_id) + 1
            

        img_file_name = str(new_id).zfill(3) + '.png'
        json_file_name =  str(new_id).zfill(3) + '_info.json'
        img_pwd = os.path.join(self.pwd_data, img_file_name)
        json_pwd = os.path.join(self.pwd_data, json_file_name)
        
        with open(json_pwd, "w") as write_file:
            json.dump(image_info, write_file)        
        
        if cv2.imwrite(img_pwd, cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)):
            rospy.loginfo("[OBJECT DETECTION] File %s save successfully to path %s", img_file_name, self.pwd_data)
            return True
        else:
            rospy.logwarn("[OBJECT DETECTION] Failed to save image %s to path %s", img_file_name, self.pwd_data)
            return False

    def detect_object(self):

        if not self.wait_for_data(5):
            return False

        px_per_mm = self.compute_px_per_mm()
        self.process_image.add_image(self.color_image, px_per_mm = px_per_mm)

        if self.settings is not None:
            self.process_image.set_settings(settings_msg_to_lib(self.settings))

        self.intrin = camera_info2intrinsics(self.color_info)

        # process image
        if self.use_truss:
            
            if not self.process_image.process_image():
                rospy.logwarn("[OBJECT DETECTION] Failed to process image")
                return False

            object_features = self.process_image.get_object_features()
            tomato_mask, peduncle_mask, _ = self.process_image.get_segments()

            cage_pose = self.generate_cage_pose(object_features['grasp_location'])
            tomatoes = self.generate_tomatoes(object_features['tomato'])
            peduncle = self.generate_peduncle(object_features['peduncle'], cage_pose) # object_features['peduncle']
            
            self.visualive_tomatoes(tomato_mask)
            self.visualive_peduncle(peduncle_mask)
            # visualive_tomatoes(self, tomato_mask)
            
            img_tomato = self.process_image.get_truss_visualization(local = True)
            img_segment = self.process_image.get_segmented_image(local = True)

        elif not self.use_truss:
            self.process_image.color_space()
            self.process_image.segment_truss()
            self.process_image.detect_tomatoes_global()
            tomato_features = self.process_image.get_tomatoes()

            cage_pose = PoseStamped()
            tomatoes = self.generate_tomatoes(tomato_features)
            peduncle = Peduncle()
            
            img_tomato = self.process_image.get_tomato_visualization(local = True)
            img_segment = self.process_image.get_segmented_image()

        truss = self.create_truss(tomatoes, cage_pose, peduncle)

        # get images
        img_hue  = self.process_image.get_color_components()
        

        # publish results tomato_img
        imgmsg_segment = self.bridge.cv2_to_imgmsg(img_segment, encoding="rgb8")
        imgmsg_tomato = self.bridge.cv2_to_imgmsg(img_tomato, encoding="rgb8")
        imgmsg_hue = self.bridge.cv2_to_imgmsg(img_hue)

        rospy.logdebug("Publishing results")
        self.pub_segment_image.publish(imgmsg_segment)
        self.pub_tomato_image.publish(imgmsg_tomato)
        self.pub_color_hue.publish(imgmsg_hue)
        self.pub_object_features.publish(truss)

        return True


    def generate_cage_pose(self, grasp_features):
        row = grasp_features['row'] 
        col = grasp_features['col']
        angle = -grasp_features['angle'] # minus since camera frame is upside down...
        rpy = [0, 0, angle]


        xyz = self.deproject(row, col)
        cage_pose =  point_to_pose_stamped(xyz, rpy, self.camera_frame, rospy.Time.now())

        return cage_pose

    def visualive_tomatoes(self, tomato_mask):
        tomato_pcl = self.segment_pcl(tomato_mask, color = (200,  50,  50, 255))
        self.pub_tomato_mask.publish(self.bridge.cv2_to_imgmsg(tomato_mask))         
        self.pub_tomato_pcl.publish(tomato_pcl)  

    def visualive_peduncle(self, peduncle_mask):
        peduncle_pcl = self.segment_pcl(peduncle_mask, color = (50, 200,  50, 255))
        self.pub_peduncle_pcl.publish(peduncle_pcl)  

    def generate_tomatoes(self, tomato_features):

        tomatoes = []
        for i in range(0, len(tomato_features['col'])):

            # Load from struct
            col = tomato_features['col'][i]
            row = tomato_features['row'][i]
            radius = tomato_features['radii'][i]

            point = self.deproject(row, col)

            depth = self.get_depth(row, col) # depth = self.depth_image[(row, col)]
            point1 = rs.rs2_deproject_pixel_to_point(self.intrin, [0,0], depth)
            point2 = rs.rs2_deproject_pixel_to_point(self.intrin, [0,radius], depth)
            radius_m = euclidean(point1, point2)

            tomatoes.append(point_to_tomato(point, radius_m, self.camera_frame))

        return tomatoes

    def generate_peduncle(self, peduncle_features, cage_pose):
        
        peduncle = Peduncle()
        peduncle.pose = cage_pose
        # peduncle.pcl = peduncle_pcl
        peduncle.radius = 0.01
        peduncle.length = 0.15
        return peduncle


    def segment_pcl(self, img, color = (255, 255, 255, 255)):
        
        r = color[0]
        g = color[1]
        b = color[2]
        a = color[3]        
        
        index = np.nonzero(img)

        uvs = list()
        for row, col in zip(index[0], index[1]):        
            uvs.append([col,row])
        
        rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                
        points = self.get_points(uvs=uvs, field_names = ("x", "y", "z"))
        for i in range(0, len(points)):
            points[i] = points[i] + (rgba,)
            
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]      
        
        return pc2.create_cloud(self.pcl.header, fields, points)        
        
          


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

        cage_pose =  point_to_pose_stamped(xyz, rpy, frame, rospy.Time.now())

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

        truss = self.create_truss(tomatoes, cage_pose, peduncle)
        self.pub_object_features.publish(truss)
        return True
        
    def compute_px_per_mm(self):
        heights = self.get_points(field_names = ("z"))
        height = np.nanmedian(np.array(heights))
        fx = self.color_info.K[0]
        fy = self.color_info.K[4]
        f = (fx + fy)/2       
        px_per_mm = f/height/1000.0
        
        rospy.loginfo('Height above table: %s [m]', height)
        rospy.loginfo('Pixels per mm: %s [px/mm]', px_per_mm)
        return px_per_mm

    def create_truss(self, tomatoes, cage_pose, peduncle):

        truss = Truss()
        truss.tomatoes = tomatoes
        truss.cage_location = cage_pose
        truss.peduncle = peduncle

        return truss

    def get_point(self, uvs):
        points = self.get_points(uvs=uvs)
        point = np.mean(points, axis = 0)
        return point
           
    def get_points(self, uvs=[], field_names = ("x", "y", "z")):
        points = list(pc2.read_points(self.pcl, skip_nans=False, field_names = field_names, uvs=uvs))   
        return points

    def deproject(self, row, col):
        # Deproject
        depth = self.get_depth(row, col)
        # rospy.logdebug("Corresponding depth: %s", self.depth_image[index])
        # https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0

        pixel = [float(col), float(row)] # [x, y]
        depth = float(depth)

        point_depth = rs.rs2_deproject_pixel_to_point(self.intrin, pixel, depth)
        
        uvs = self.gen_patch(row, col)
        point_pcl = self.get_point(uvs)        
        rospy.logdebug("Point based on deprojection: %s", point_depth)
        rospy.logdebug("Point obtained from point cloud: %s", point_pcl)        
        
        
        point = point_depth
        return point

    def gen_patch(self, row, col):
        patch_width = (self.patch_size - 1)/2

        dim = self.depth_image.shape
        H = dim[0]
        W = dim[1]

        row_start = max([row - patch_width, 0])
        row_end = min([row + patch_width, H - 1])

        col_start = max([col - patch_width, 0])
        col_end = min([col + patch_width, W - 1])

        rows = np.arange(row_start, row_end + 1)
        cols = np.arange(col_start, col_end + 1)
        
        uvs = list() # [col, row]
        
        for col in cols:
            for row in rows:
                uvs.append([col, row])
                
        return uvs

    def get_depth(self, row, col):
        patch_width = (self.patch_size - 1)/2

        dim = self.depth_image.shape
        H = dim[0]
        W = dim[1]

        row_start = max([row - patch_width, 0])
        row_end = min([row + patch_width, H - 1])

        col_start = max([col - patch_width, 0])
        col_end = min([col + patch_width, W - 1])

        rows = np.arange(row_start, row_end + 1)
        cols = np.arange(col_start, col_end + 1)

        depth_patch = self.depth_image[rows[:, np.newaxis], cols]
        non_zero = np.nonzero(depth_patch)
        depth_patch_non_zero = depth_patch[non_zero]

        return np.mean(depth_patch_non_zero)

    def take_action(self):
        success = None
        msg = String()

        if (self.event == "detect_tomato"):
            rospy.logdebug("[OBEJCT DETECTION] Detect tomato")
            # if not self.debug_mode:
            #     success = self.generate_object()
            # if self.debug_mode:
            self.process_image.use_truss = False
            self.use_truss = False
            self.take_picture = True
            success = self.detect_object()

        elif (self.event == "detect_truss"):
            rospy.logdebug("[OBEJCT DETECTION] Detect truss")
            self.process_image.use_truss = True
            self.use_truss = True
            self.take_picture = True
            success = self.detect_object()

        elif (self.event == "save_image"):
            rospy.logdebug("[OBEJCT DETECTION] Take picture")
            self.take_picture = True
            success = self.save_image()

        elif (self.event == "e_init"):
            self.init = True
            success = True

        # publish success
        if success is not None:
            self.clear_all_data()
            if success == True:
                msg.data = "e_success"
                self.event = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("[OBEJCT DETECTION] failed to execute command %s", self.event)
                self.event = None

            self.pub_e_out.publish(msg)

def euclidean(v1, v2):
    return sum((p-q)**2 for p, q in zip(v1, v2)) ** .5


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
