#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: jelle
"""

import numpy as np
import rospy
import cv2
import json
import os
from cv_bridge import CvBridge, CvBridgeError

# msg
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
from flex_grasp.msg import ImageProcessingSettings
from flex_grasp.msg import FlexGraspErrorCodes

from flex_shared_resources.depth_interface import DepthImageFilter, PointCloudFilter
from flex_vision.detect_truss.ProcessImage import ProcessImage
from flex_shared_resources.data_logger import DataLogger
from flex_shared_resources.experiment_info import ExperimentInfo

from flex_shared_resources.utils.conversions import point_to_pose_stamped, settings_lib_to_msg, settings_msg_to_lib
from func.utils import camera_info2rs_intrinsics
from func.utils import colored_depth_image

DEFAULT_CAMERA_SIM = False

class ObjectDetection(object):
    """ObjectDetection"""

    def __init__(self, node_name, playback=False):
        self.node_name = node_name

        # state
        self.take_picture = False
        self.playback = playback
        if self.playback:
            rospy.loginfo("[{0}] Object detection launched in playback mode!".format(self.node_name))

        # params
        self.camera_sim = rospy.get_param("camera_sim", DEFAULT_CAMERA_SIM)
        self.process_image = ProcessImage(name='ros_tomato', pwd='', save=False)

        # frames
        self.camera_frame = "camera_color_optical_frame"

        # data input
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None

        self.input_logger = self.initialize_input_logger()
        self.output_logger = self.initialize_output_logger()
        self.settings_logger = DataLogger(self.node_name, {"settings": "image_processing_settings"}, {"settings": ImageProcessingSettings}, bag_name='image_processing_settings')

        # cv bridge
        self.bridge = CvBridge()

        # params
        self.patch_size = 5
        self.peduncle_height = 0.080  # [m]
        self.settings = settings_lib_to_msg(self.process_image.get_settings())

        self.experiment_info = ExperimentInfo(self.node_name)

        pub_image_processing_settings = rospy.Publisher("image_processing_settings",
                                                        ImageProcessingSettings, queue_size=10, latch=True)

        pub_image_processing_settings.publish(self.settings)

        # TODO: log settings!
        rospy.Subscriber("image_processing_settings", ImageProcessingSettings, self.image_processing_settings_cb)

    def initialize_input_logger(self):
        # inputs
        topics_in = {'color_image': 'camera/color/image_raw',
                     'depth_image': 'camera/aligned_depth_to_color/image_raw',
                     'camera_info': 'camera/color/camera_info',
                     'pcl': 'camera/depth_registered/points'}

        types_in = {'color_image': Image,
                    'depth_image': Image,
                    'camera_info': CameraInfo,
                    'pcl': PointCloud2}

        callbacks = {'color_image': self.color_image_cb,
                     'depth_image': self.depth_image_cb,
                     'camera_info': self.camera_info_cb,
                     'pcl': self.point_cloud_cb}

        for key in types_in:
            rospy.Subscriber(topics_in[key], types_in[key], callbacks[key])

        return DataLogger(self.node_name, topics_in, types_in, bag_name='camera')

    def initialize_output_logger(self):
        # outputs
        topics_out = {'truss_pose': 'truss_pose',
                      'tomato_image': 'tomato_image',
                      'tomato_image_total': 'tomato_image_total',
                      'depth_image': 'depth_image'}

        types_out = {'truss_pose': PoseStamped,
                     'tomato_image': Image,
                     'tomato_image_total': Image,
                     'depth_image': Image}

        return DataLogger(self.node_name, topics_out, types_out, bag_name=self.node_name)

    def color_image_cb(self, msg):
        if (self.color_image is None) and self.take_picture:
            rospy.logdebug("[{0}] Received color image message".format(self.node_name))
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def depth_image_cb(self, msg):
        if (self.depth_image is None) and self.take_picture:
            rospy.logdebug("[{0}] Received depth image message".format(self.node_name))
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough") / 1000.0
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def camera_info_cb(self, msg):
        if (self.camera_info is None) and self.take_picture:
            rospy.logdebug("[{0}] Received camera info message".format(self.node_name))
            self.camera_info = msg

    def point_cloud_cb(self, msg, force=False):
        if (self.pcl is None) and self.take_picture:
            rospy.logdebug("[{0}] Received point cloud info message".format(self.node_name))
            self.pcl = msg

    def image_processing_settings_cb(self, msg):
        self.settings = msg
        rospy.logdebug("[{0}] Received image processing settings".format(self.node_name))

    def received_messages(self):
        """Returns a dictionary which contains information about what data has been received"""
        is_received = {'color_image': self.color_image is not None,
                       'depth_image': self.depth_image is not None,
                       'camera_info': self.camera_info is not None,
                       'pcl': self.pcl is not None,
                       'all': True}

        for key in is_received:
            is_received['all'] = is_received['all'] and is_received[key]

        return is_received

    def print_received_messages(self, is_received):
        """Prints a warning for the data which has not been received"""
        for key in is_received:
            if not is_received[key]:
                rospy.logwarn("[{0}] Did not receive {1} data yet.".format(self.node_name, key))

    def wait_for_messages(self, timeout=1):
        start_time = rospy.get_time()
        is_received = {}

        while rospy.get_time() - start_time < timeout:
            is_received = self.received_messages()
            if is_received['all']:
                self.take_picture = False
                rospy.logdebug("[{0}] Received all data".format(self.node_name))
                return True

            rospy.sleep(0.1)

        self.print_received_messages(is_received)
        return False

    def save_data(self, result_img=None):
        """Save visual data"""

        # information about the image which will be stored
        image_info = {'px_per_mm': self.compute_px_per_mm()}
        json_pwd = os.path.join(self.experiment_info.path, self.experiment_info.id + '_info.json')

        rgb_img = self.color_image
        depth_img = colored_depth_image(self.depth_image.copy())

        with open(json_pwd, "w") as write_file:
            json.dump(image_info, write_file)

        self.save_image(rgb_img, self.experiment_info.path,  name=self.experiment_info.id + '_rgb.png')
        self.save_image(depth_img, self.experiment_info.path, name=self.experiment_info.id + '_depth.png')
        if result_img is not None:
            self.save_image(result_img, self.experiment_info.path, name=self.experiment_info.id + '_result.png')

        return FlexGraspErrorCodes.SUCCESS

    def save_image(self, img, pwd, name):
        """Save an RGB image to the given path, if the path does not exist create it."""
        full_pwd = os.path.join(pwd, name)

        # Make sure the folder exists
        if not os.path.isdir(pwd):
            rospy.loginfo("[{0}]New path, creating a new folder {1}".format(self.node_name, pwd))
            os.mkdir(pwd)

        # OpenCV assumes the image to be BGR
        if cv2.imwrite(full_pwd, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)):
            rospy.logdebug("[{0}] Successfully saved image to path {1}".format(self.node_name, full_pwd))
            return FlexGraspErrorCodes.SUCCESS
        else:
            rospy.logwarn("[{0}] Failed to save image to path %s".format(self.node_name, full_pwd))
            return FlexGraspErrorCodes.FAILURE

    def detect_object(self):
        """Detect object"""

        if self.playback:
            rospy.loginfo("[{0}] Playback is active: publishing object_detection messages from bag!".format(self.node_name))
            success = self.output_logger.publish_messages_from_bag(self.experiment_info.path, self.experiment_info.id)
            return success

        self.settings_logger.write_messages_to_bag({"settings": self.settings}, self.experiment_info.path, self.experiment_info.id)

        px_per_mm = self.compute_px_per_mm()
        self.process_image.add_image(self.color_image, px_per_mm=px_per_mm)

        if self.settings is not None:
            self.process_image.set_settings(settings_msg_to_lib(self.settings))

        if not self.process_image.process_image():
            rospy.logwarn("[OBJECT DETECTION] Failed to process image")
            self.save_data()
            return FlexGraspErrorCodes.FAILURE

        object_features = self.process_image.get_object_features()
        tomato_mask, peduncle_mask, _ = self.process_image.get_segments()
        truss_visualization = self.process_image.get_truss_visualization(local=True)
        truss_visualization_total = self.process_image.get_truss_visualization(local=False)

        json_pwd = os.path.join(self.experiment_info.path, self.experiment_info.id, 'truss_features.json')
        with open(json_pwd, 'w') as outfile:
            json.dump(object_features, outfile)
        self.save_data(result_img=truss_visualization)

        depth_img = colored_depth_image(self.depth_image.copy())

        # publish results
        # TODO: also publish result in global frame!
        output_messages = {}
        output_messages['depth_image'] = self.bridge.cv2_to_imgmsg(depth_img, encoding="rgb8")
        output_messages['tomato_image'] = self.bridge.cv2_to_imgmsg(truss_visualization, encoding="rgba8")
        output_messages['tomato_image_total'] = self.bridge.cv2_to_imgmsg(truss_visualization_total, encoding="rgba8")
        output_messages['truss_pose'] = self.generate_cage_pose(object_features['grasp_location'], peduncle_mask)

        success = self.output_logger.publish_messages(output_messages, self.experiment_info.path, self.experiment_info.id)
        return success

    def generate_cage_pose(self, grasp_features, peduncle_mask):
        row = grasp_features['row']
        col = grasp_features['col']
        angle = grasp_features['angle']
        if angle is None:
            rospy.logwarn("Failed to compute caging pose: object detection returned None!")
            return False

        # orientation
        rospy.logdebug("[{0}] Object angle in degree {1}".format(self.node_name, np.rad2deg(angle)))
        rpy = [0, 0, angle]

        rs_intrinsics = camera_info2rs_intrinsics(self.camera_info)
        depth_image_filter = DepthImageFilter(self.depth_image, rs_intrinsics, patch_size=5, node_name=self.node_name)
        depth_assumption = self.get_table_height() - self.peduncle_height
        depth_measured = depth_image_filter.get_depth(row, col)

        # location
        rospy.loginfo("[{0}] Depth based on assumptions: {1:0.3f} [m]".format(self.node_name, depth_assumption))
        rospy.loginfo("[{0}] Depth measured: {1:0.3f} [m]".format(self.node_name, depth_measured))

        xyz = depth_image_filter.deproject(row, col, depth_assumption)

        if np.isnan(xyz).any():
            rospy.logwarn("[{0}] Failed to compute caging pose, will try based on segment!".format(self.node_name))
            xyz = depth_image_filter.deproject(row, col, segment=peduncle_mask)

            if np.isnan(xyz).any():
                rospy.logwarn("[{0}] Failed to compute caging pose!".format(self.node_name))
                return False

        return point_to_pose_stamped(xyz, rpy, self.camera_frame, rospy.Time.now())

    def get_table_height(self):
        """Estimate the distance between the camera and table"""
        point_cloud_filter = PointCloudFilter(self.pcl, patch_size=5, node_name=self.node_name)
        heights = point_cloud_filter.get_points(field_names="z")
        return np.nanmedian(np.array(heights))

    def compute_px_per_mm(self):
        """Estimate the amount of pixels per mm"""
        height = self.get_table_height()
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        f = (fx + fy) / 2
        px_per_mm = f / height / 1000.0

        rospy.logdebug('[{0}] Distance between camera and table: {1:0.3f} [m]'.format(self.node_name, height))
        rospy.logdebug('[{0}] Pixels per mm: {1:0.3f} [px/mm]'.format(self.node_name, px_per_mm))
        return px_per_mm

    def collect_messages(self):
        """When method is called the class will start collecting required messages"""
        self.take_picture = True
        if self.playback:
            rospy.loginfo("[{0}] Playback is active: publishing camera messages from bag!".format(self.node_name))
            self.input_logger.publish_messages_from_bag(self.experiment_info.path, self.experiment_info.id)

    def log_input_messages(self):
        """"log messages"""
        if self.playback:
            rospy.logdebug("[{0}] Will not logging input messages: running in playback mode!".format(self.node_name))
        else:
            rospy.logdebug("[{0}] Logging input messages".format(self.node_name))
            self.input_logger.write_messages_to_bag(self.get_messages(), self.experiment_info.path, self.experiment_info.id)

    def get_messages(self):
        messages = {}
        if self.color_image is not None:
            messages['color_image'] = self.bridge.cv2_to_imgmsg(self.color_image, encoding="rgb8")
        else:
            rospy.logwarn("[{0}] Failed to get color_image message".format(self.node_name))
            messages['color_image'] = None
        if self.depth_image is not None:
            messages['depth_image'] = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="passthrough")
        else:
            rospy.logwarn("[{0}] Failed to get depth_image message".format(self.node_name))
            messages['depth_image'] = None

        messages['camera_info'] = self.camera_info
        messages['pcl'] = self.pcl
        return messages

    def reset(self):
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None