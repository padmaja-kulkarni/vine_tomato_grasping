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
import pyrealsense2 as rs
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

from flex_grasp.msg import Tomato, Truss, Peduncle, ImageProcessingSettings

from flex_vision.detect_truss.ProcessImage import ProcessImage

from flex_shared_resources.utils.conversions import point_to_pose_stamped, settings_lib_to_msg, settings_msg_to_lib
from func.utils import camera_info2intrinsics
from func.utils import colored_depth_image


from flex_grasp.msg import FlexGraspErrorCodes
from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log

import os  # os.sep


class ObjectDetection(object):
    """ObjectDetection"""
    node_name = "OBJECT DETECTION"

    def __init__(self):

        # data
        self.color_image = None
        self.depth_image = None
        self.depth_info = None
        self.color_info = None
        self.pcl = None
        self.trans = None

        # state
        self.command = None
        self.init = None
        self.take_picture = False

        # frames
        self.color_frame = "camera_color_optical_frame"
        self.depth_frame = "camera_depth_optical_frame"
        self.camera_frame = "camera_color_optical_frame"

        # params
        self.camera_sim = rospy.get_param("camera_sim")
        self.debug_mode = rospy.get_param("object_detection/debug")
        self.patch_size = 5
        self.surface_height = 0.019  # [m]
        self.peduncle_height = 0.080  # [m]

        self.bridge = CvBridge()

        self.pwd_current = os.path.dirname(__file__)  # path to THIS file
        self.data_set = 'default'
        self.pwd_data = os.path.join(os.getcwd(), 'thesis_data', self.data_set)

        rospy.loginfo("Storing visual results in: ", self.pwd_data)

        self.process_image = ProcessImage(name='ros_tomato', pwd='', save=False)

        settings = settings_lib_to_msg(self.process_image.get_settings())

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[OBJECT DETECTION] Launching object detection node in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("object_detection", anonymous=True, log_level=log_level)

        # Publish
        self.pub_truss_pose = rospy.Publisher("truss_pose", PoseStamped, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         FlexGraspErrorCodes, queue_size=10, latch=True)

        self.pub_object_features = rospy.Publisher("object_features",
                                                   Truss, queue_size=5, latch=True)

        self.pub_tomato_image = rospy.Publisher("tomato_image",
                                                Image, queue_size=5, latch=True)

        self.pub_depth_image = rospy.Publisher("depth_image",
                                               Image, queue_size=5, latch=True)

        self.pub_color_hue = rospy.Publisher("color_hue",
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
        rospy.Subscriber("camera/color/image_raw", Image, self.color_image_cb)
        rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, self.depth_image_cb)
        rospy.Subscriber("camera/color/camera_info", CameraInfo, self.color_info_cb)
        rospy.Subscriber("camera/aligned_depth_to_color/camera_info", CameraInfo, self.depth_info_cb)
        rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.point_cloud_cb)
        rospy.Subscriber("experiment_pwd", String, self.data_set_cb)
        rospy.Subscriber("image_processing_settings", ImageProcessingSettings, self.image_processing_settings_cb)

    def data_set_cb(self, msg):
        if self.data_set != msg.data:
            self.data_set = msg.data
            self.pwd_data = self.data_set
            # self.pwd_data = os.path.join(self.pwd_current, '..', '..', 'detect_truss', 'src', 'data', self.data_set)
            rospy.loginfo("[INFO] Storing results in folder %s", self.data_set)

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[OBJECT DETECTION] Received object detection command: %s", self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)

    def color_image_cb(self, msg):
        if (self.color_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received color image message")
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                print(e)

    def depth_image_cb(self, msg):
        if (self.depth_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received depth image message")
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough") / 1000.0
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

    def log_image(self, result_img=None):

        if not self.wait_for_data(5):
            return FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        pwd_1 = os.path.join(os.sep, *self.pwd_data.split(os.sep)[0:-1])
        if not os.path.isdir(pwd_1):
            rospy.loginfo("New path, creating a new folder: " + pwd_1)
            os.mkdir(pwd_1)

        if not os.path.isdir(self.pwd_data):
            rospy.loginfo("New path, creating a new folder: " + self.pwd_data)
            os.mkdir(self.pwd_data)

            # imaformation about the image which will be stored
        image_info = {}
        image_info['px_per_mm'] = self.compute_px_per_mm()

        # check contents folder
        contents = os.listdir(self.pwd_data)

        # determine starting number
        if len(contents) == 0:
            id_int = 1
        else:
            contents.sort()
            file_name = contents[-1]
            file_id = file_name[:3]
            id_int = int(file_id) + 1
        id_string = str(id_int).zfill(3)

        name_rgb = id_string + '_rgb.png'
        name_depth = id_string + '_depth.png'
        name_result = id_string + '_result.png'
        json_file_name = id_string + '_info.json'
        json_pwd = os.path.join(self.pwd_data, json_file_name)

        rgb_img = self.color_image
        depth_img = colored_depth_image(self.depth_image.copy())

        with open(json_pwd, "w") as write_file:
            json.dump(image_info, write_file)

        result = self.save_image(rgb_img, self.pwd_data, name_rgb)
        result = self.save_image(depth_img, self.pwd_data, name_depth)
        if result_img is not None:
            result = self.save_image(result_img, self.pwd_data, name_result)

        if result == FlexGraspErrorCodes.SUCCESS:
            rospy.loginfo("[OBJECT DETECTION] Succesfully logged data of id %s", id_string)

        imgmsg_depth = self.bridge.cv2_to_imgmsg(depth_img, encoding="rgb8")
        self.pub_depth_image.publish(imgmsg_depth)
        return result

    def save_image(self, img, pwd, name):
        full_pwd = os.path.join(pwd, name)
        if cv2.imwrite(full_pwd, cv2.cvtColor(img, cv2.COLOR_RGB2BGR)):
            rospy.logdebug("[OBJECT DETECTION] File %s save successfully to path %s", name, self.pwd_data)
            return FlexGraspErrorCodes.SUCCESS
        else:
            rospy.logwarn("[OBJECT DETECTION] Failed to save image %s to path %s", name, self.pwd_data)
            return FlexGraspErrorCodes.FAILURE

    def detect_object(self):

        if not self.wait_for_data(5):
            return FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        px_per_mm = self.compute_px_per_mm()
        self.process_image.add_image(self.color_image, px_per_mm=px_per_mm)

        if self.settings is not None:
            self.process_image.set_settings(settings_msg_to_lib(self.settings))

        self.intrin = camera_info2intrinsics(self.color_info)

        # process image
        if self.use_truss:

            if not self.process_image.process_image():
                rospy.logwarn("[OBJECT DETECTION] Failed to process image")
                return FlexGraspErrorCodes.FAILURE

            object_features = self.process_image.get_object_features()
            tomato_mask, peduncle_mask, _ = self.process_image.get_segments()

            cage_pose = self.generate_cage_pose(object_features['grasp_location'], peduncle_mask)
            tomatoes = self.generate_tomatoes(object_features['tomato'])
            peduncle = self.generate_peduncle(object_features['peduncle'], cage_pose)  # object_features['peduncle']

            self.visualive_tomatoes(tomato_mask)
            self.visualive_peduncle(peduncle_mask)

            img_tomato = self.process_image.get_truss_visualization(local=True)

        elif not self.use_truss:
            self.process_image.color_space()
            self.process_image.segment_truss()
            self.process_image.detect_tomatoes_global()
            tomato_features = self.process_image.get_tomatoes()

            cage_pose = PoseStamped()
            tomatoes = self.generate_tomatoes(tomato_features)
            peduncle = Peduncle()

            img_tomato = self.process_image.get_tomato_visualization(local=True)

        truss = self.create_truss(tomatoes, cage_pose, peduncle)

        # get images
        img_hue = self.process_image.get_color_components()
        img_depth = colored_depth_image(self.depth_image.copy())

        self.log_image(result_img=img_tomato)

        # publish results tomato_img
        imgmsg_tomato = self.bridge.cv2_to_imgmsg(img_tomato, encoding="rgba8")
        imgmsg_depth = self.bridge.cv2_to_imgmsg(img_depth, encoding="rgb8")
        imgmsg_hue = self.bridge.cv2_to_imgmsg(img_hue)

        rospy.logdebug("Publishing results")

        self.pub_tomato_image.publish(imgmsg_tomato)
        self.pub_depth_image.publish(imgmsg_depth)
        self.pub_color_hue.publish(imgmsg_hue)
        if cage_pose is False:
            return FlexGraspErrorCodes.FAILURE
        self.pub_object_features.publish(truss)
        self.pub_truss_pose.publish(cage_pose)
        return FlexGraspErrorCodes.SUCCESS

    def generate_cage_pose(self, grasp_features, peduncle_mask):
        row = grasp_features['row']
        col = grasp_features['col']
        angle = grasp_features['angle']
        if angle is None:
            rospy.logwarn("Failed to compute caging pose: object detection returned None!")
            return False

        rospy.logdebug('angle: %s', np.rad2deg(angle))
        rpy = [0, 0, angle]


        table_height = self.get_table_height()
        depth = 0.47 #  self.get_table_height() - self.peduncle_height


        rospy.loginfo("Table height: %s", table_height)
        rospy.loginfo("Peduncle height: %s", self.peduncle_height)
        rospy.loginfo("Depth based on assumptions: %s", depth)
        rospy.loginfo("Depth measured: %s", self.get_depth(row, col))
        xyz = self.deproject(row, col, depth=depth)

        if np.isnan(xyz).any():
            rospy.logwarn("Failed to compute caging pose, will try based on segment!")
            xyz = self.deproject(row, col, segment=peduncle_mask)

            if np.isnan(xyz).any():
                rospy.loginfo("Failed to compute caging pose!")
                return False

        cage_pose = point_to_pose_stamped(xyz, rpy, self.camera_frame, rospy.Time.now())

        return cage_pose

    def visualive_tomatoes(self, tomato_mask):
        tomato_pcl = self.segment_pcl(tomato_mask, color=(200, 50, 50, 255))
        # self.pub_tomato_mask.publish(self.bridge.cv2_to_imgmsg(tomato_mask))         
        self.pub_tomato_pcl.publish(tomato_pcl)

    def visualive_peduncle(self, peduncle_mask):
        peduncle_pcl = self.segment_pcl(peduncle_mask, color=(50, 200, 50, 255))
        self.pub_peduncle_pcl.publish(peduncle_pcl)

    def generate_tomatoes(self, tomato_features):

        tomatoes = []
        for i in range(0, len(tomato_features['col'])):
            # Load from struct
            col = tomato_features['col'][i]
            row = tomato_features['row'][i]
            radius = tomato_features['radii'][i]

            point = self.deproject(row, col)

            depth = self.get_depth(row, col)  # depth = self.depth_image[(row, col)]
            point1 = rs.rs2_deproject_pixel_to_point(self.intrin, [0, 0], depth)
            point2 = rs.rs2_deproject_pixel_to_point(self.intrin, [0, radius], depth)
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

    def segment_pcl(self, img, color=(255, 255, 255, 255)):

        r = color[0]
        g = color[1]
        b = color[2]
        a = color[3]

        index = np.nonzero(img)

        uvs = list()
        for row, col in zip(index[0], index[1]):
            uvs.append([col, row])

        rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

        points = self.get_points(uvs=uvs, field_names=("x", "y", "z"))
        for i in range(0, len(points)):
            points[i] = points[i] + (rgba,)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]

        return pc2.create_cloud(self.pcl.header, fields, points)

    def generate_object(self):

        # %%##################
        ### Cage location ###
        #####################
        table_height = 0.23
        frame = "world"
        object_x = rospy.get_param("object_x")
        object_y = rospy.get_param("object_y")
        angle = rospy.get_param("object_angle")
        xyz = [object_x, object_y, 0.05 + table_height]
        rpy = [3.1415, 0, angle]  # 3.1415/2.0

        cage_pose = point_to_pose_stamped(xyz, rpy, frame, rospy.Time.now())

        # %%#############
        ### Peduncle ###
        ################
        L = 0.15
        peduncle = Peduncle()
        peduncle.pose = cage_pose
        peduncle.radius = 0.005
        peduncle.length = L

        # %%#############
        ### tomatoes ###
        ################
        radii = [0.05, 0.05]
        t1x = xyz[0] + (L / 2 + radii[0]) * np.cos(angle)
        t1y = xyz[1] - (L / 2 + radii[0]) * np.sin(angle)
        t2x = xyz[0] - (L / 2 + radii[1]) * np.cos(angle)
        t2y = xyz[1] + (L / 2 + radii[1]) * np.sin(angle)
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

    def get_table_height(self):
        heights = self.get_points(field_names=("z"))
        return np.nanmedian(np.array(heights))

    def compute_px_per_mm(self):
        height = self.get_table_height()
        fx = self.color_info.K[0]
        fy = self.color_info.K[4]
        f = (fx + fy) / 2
        px_per_mm = f / height / 1000.0

        rospy.logdebug('Height above table: %s [m]', height)
        rospy.logdebug('Pixels per mm: %s [px/mm]', px_per_mm)
        return px_per_mm

    def create_truss(self, tomatoes, cage_pose, peduncle):

        truss = Truss()
        truss.tomatoes = tomatoes
        truss.cage_location = cage_pose
        truss.peduncle = peduncle

        return truss

    def get_point(self, uvs):
        points = self.get_points(uvs=uvs)
        point = np.mean(points, axis=0)
        return point

    def get_points(self, uvs=[], field_names=("x", "y", "z")):
        points = list(pc2.read_points(self.pcl, skip_nans=False, field_names=field_names, uvs=uvs))
        return points

    def deproject(self, row, col, depth=None, segment=None):
        # TODO: these should never be floats!
        row = int(row)
        col = int(col)
        
        if depth is None:
            depth = self.get_depth(row, col, segment=segment)

        if np.isnan(depth):
            rospy.logwarn("[OBJECT DETECTION] Computed depth is nan, can not compute point!")
            return 3 * [np.nan]

        # https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
        pixel = [float(col), float(row)]  # [x, y]
        depth = float(depth)
        point_depth = rs.rs2_deproject_pixel_to_point(self.intrin, pixel, depth)

        uvs = self.gen_patch(row, col)
        point_pcl = self.get_point(uvs)

        rospy.logdebug("Point based on deprojection: %s", point_depth)
        rospy.logdebug("Point obtained from point cloud: %s", point_pcl)

        point = point_depth
        return point

    def gen_patch(self, row, col):
        patch_width = (self.patch_size - 1) / 2

        dim = self.depth_image.shape
        H = dim[0]
        W = dim[1]

        row_start = max([row - patch_width, 0])
        row_end = min([row + patch_width, H - 1])

        col_start = max([col - patch_width, 0])
        col_end = min([col + patch_width, W - 1])

        rows = np.arange(row_start, row_end + 1)
        cols = np.arange(col_start, col_end + 1)

        uvs = list()  # [col, row]

        for col in cols:
            for row in rows:
                uvs.append([col, row])

        return uvs

    def get_depth(self, row, col, segment=None):

        # TODO: these should never be floats!
        row = int(row)
        col = int(col)

        if segment is None:
            patch_width = int((self.patch_size - 1) / 2)

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
        else:
            depth_patch = self.depth_image[segment > 0]

        non_zero = np.nonzero(depth_patch)
        depth_patch_non_zero = depth_patch[non_zero]

        return np.median(depth_patch_non_zero)

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        if (self.command == "detect_tomato"):
            rospy.logdebug("[OBEJCT DETECTION] Detect tomato")
            # if not self.debug_mode:
            #     success = self.generate_object()
            # if self.debug_mode:
            self.process_image.use_truss = False
            self.use_truss = False
            self.take_picture = True
            result = self.detect_object()

        elif (self.command == "detect_truss"):
            rospy.logdebug("[OBEJCT DETECTION] Detect truss")
            self.process_image.use_truss = True
            self.use_truss = True
            self.take_picture = True
            result = self.detect_object()

        elif (self.command == "save_image"):
            rospy.logdebug("[OBEJCT DETECTION] Take picture")
            self.take_picture = True
            result = self.log_image()

        elif (self.command == "e_init"):
            self.init = True
            result = FlexGraspErrorCodes.SUCCESS

        elif self.command is not None:
            result = FlexGraspErrorCodes.UNKNOWN_COMMAND

        # publish success
        if result is not None:
            msg.val = result
            flex_grasp_error_log(result, self.node_name)
            self.pub_e_out.publish(msg)
            self.clear_all_data()
            self.command = None


def euclidean(v1, v2):
    return sum((p - q) ** 2 for p, q in zip(v1, v2)) ** .5


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
