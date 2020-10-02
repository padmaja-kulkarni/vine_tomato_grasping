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
from func.flex_grasp_error import flex_grasp_error_log

# custom functions
from func.conversions import list_to_position, list_to_orientation

from flex_grasp.msg import FlexGraspErrorCodes
from communication import Communication


class Calibration(object):
    """Calibration"""
    node_name = "CALIBRATION"    
    
    def __init__(self):

#        robot_name = rospy.get_param("robot_name") 
#        print(robot_name)
        self.debug_mode = rospy.get_param("calibrate/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
        else:
            log_level = rospy.INFO

        rospy.init_node("calibrate",
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


        self.calibration_frame = rospy.get_param('/px150/robot_base_frame')# "px150/base_link"
        self.planning_frame = rospy.get_param('/px150/planning_frame')
        self.pose_array = None

        self.command = None

        # we need to be sucsribed to the aruco tracker for it to publish the tf
        rospy.Subscriber("/px150/aruco_tracker/pose", PoseStamped, self.aruco_tracker_cb)

        self.pub_e_out = rospy.Publisher("~e_out",
                                     FlexGraspErrorCodes, queue_size=10, latch=True)

        self.pub_move_robot_pose = rospy.Publisher("/px150/robot_pose",
                                  PoseStamped, queue_size=10, latch=False)

        self.pub_pose_array = rospy.Publisher("/px150/pose_array",
                                PoseArray, queue_size=5, latch=True)

        move_robot_topic = "/px150/move_robot"
        self.move_robot_communication = Communication(move_robot_topic, timeout = 15)   

        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[CALIBTRATION] Received event message: %s", self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg) 


    def aruco_tracker_cb(self, msg):
        pass

    def init_poses_1(self):
        r_amplitude = 0.00
        z_amplitude = 0.00

        r_min = 0.24
        z_min = 0.28 # 0.05

        pos_intervals = 1
        if pos_intervals == 1:
            r_vec = [r_min + r_amplitude] # np.linspace(x_min, x_min + 2*x_amplitude, 2) # 
            z_vec = [z_min + z_amplitude]
        else:
            r_vec = np.linspace(r_min, r_min + 2*r_amplitude, pos_intervals)
            z_vec = np.linspace(z_min, z_min + 2*z_amplitude, pos_intervals)

        ai_amplitude = np.deg2rad(38.0)
        aj_amplitude = np.deg2rad(38.0)
        ak_amplitude = np.deg2rad(15.0)

        rot_intervals = 2
        ai_vec = np.linspace(-ai_amplitude, ai_amplitude, rot_intervals)
        aj_vec = np.linspace(-aj_amplitude, aj_amplitude, rot_intervals)
        ak_vec = [-ak_amplitude, ak_amplitude]

        return self.generate_poses(r_vec, z_vec, ai_vec, aj_vec, ak_vec)

    def init_poses_2(self):

        surface_height = 0.019
        height_finger = 0.040  # [m]
        finger_link2ee_link = 0.023  # [m]
        grasp_height = height_finger + finger_link2ee_link - surface_height


        sag_angle = np.deg2rad(6.0) # [deg]
        r_amplitude = 0.08
        z_amplitude = 0.00

        r_min = 0.10
        z_min = grasp_height  # 0.05

        pos_intervals = 3
        if pos_intervals == 1:
            r_vec = [r_min + r_amplitude]  # np.linspace(x_min, x_min + 2*x_amplitude, 2) #
            z_vec = [z_min + z_amplitude]
        else:
            r_vec = np.linspace(r_min, r_min + 2 * r_amplitude, pos_intervals)
            z_vec = np.linspace(z_min, z_min + 2 * z_amplitude, pos_intervals) + np.tan(sag_angle)*r_vec

        ak_amplitude = np.deg2rad(15.0)

        rot_intervals = 2
        ai_vec = [np.deg2rad(0)]
        aj_vec = [np.deg2rad(90)]
        ak_vec = [-ak_amplitude, ak_amplitude]
        return self.generate_poses_2(r_vec, z_vec, ai_vec, aj_vec, ak_vec)

    def generate_poses(self, r_vec, z_vec, ai_vec, aj_vec, ak_vec):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.calibration_frame
        pose_array.header.stamp = rospy.Time.now()

        poses = []
        for ak in ak_vec:
            for r in r_vec:
                for z in z_vec:
                    for aj in aj_vec:
                        for ai in ai_vec:
                            pose = Pose()

                            x = r * np.cos(ak)
                            y = r * np.sin(ak)
                            pose.position = list_to_position([x, y, z])

                            pose.orientation = list_to_orientation([ai, aj, ak])

                            poses.append(pose)

        pose_array.poses = poses

        self.pub_pose_array.publish(pose_array)
        self.pose_array = pose_array
        return FlexGraspErrorCodes.SUCCESS


    def generate_poses_2(self, r_vec, z_vec, ai_vec, aj_vec, ak_vec):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.calibration_frame
        pose_array.header.stamp = rospy.Time.now()

        poses = []
        for ak in ak_vec:
            for r, z in zip(r_vec, z_vec):
                for aj in aj_vec:
                    for ai in ai_vec:
                        pose = Pose()

                        x = r * np.cos(ak)
                        y = r * np.sin(ak)
                        pose.position = list_to_position([x, y, z])

                        pose.orientation = list_to_orientation([ai, aj, ak])

                        poses.append(pose)

        pose_array.poses = poses

        self.pub_pose_array.publish(pose_array)
        self.pose_array = pose_array
        return FlexGraspErrorCodes.SUCCESS



    def calibrate(self, track_marker=True):

        # does pose array contain something?
        if self.pose_array is None:
            rospy.logwarn("[CALIBRATE] pose_array is still empty")
            return FlexGraspErrorCodes.REQUIRED_DATA_MISSING

        try:
            trans = self.tfBuffer.lookup_transform(self.planning_frame,self.pose_array.header.frame_id,  rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[CALIBRATE] failed to get transform from %s to %s", self.pose_array.header.frame_id, self.planning_frame)
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        result = self.move_robot_communication.wait_for_result("reset")

        for pose in self.pose_array.poses:
            if rospy.is_shutdown():
                return FlexGraspErrorCodes.SHUTDOWN

            pose_stamped = PoseStamped()
            pose_stamped.header = self.pose_array.header
            pose_stamped.pose = pose

            # transform to planning frame
            pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
        
            self.pub_move_robot_pose.publish(pose_trans)
            result = self.move_robot_communication.wait_for_result("move_manipulator") # timout = 5?

            if result == FlexGraspErrorCodes.SUCCESS:
                if track_marker:
                    # camera delay + wait a small amount of time for vibrations to stop
                    rospy.sleep(1.5)
                    try:
                        self.client.take_sample()
                    except:
                        rospy.logwarn("[CALIBRATE] Failed to take sample, marker might not be visible.")
                    
            elif result == FlexGraspErrorCodes.DYNAMIXEL_ERROR:
                print('dynamixel error triggered, returning error')
                return result
            elif result == FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR:
                print('dynamixel error triggered, returning error')
                return result

        # reset
        result = self.move_robot_communication.wait_for_result("home") 
        
        # compute calibration transform
        if not track_marker:
            return FlexGraspErrorCodes.SUCCESS
        else:
            calibration_transform = self.client.compute_calibration()

            if calibration_transform.valid:
                rospy.loginfo("[CALIBRATE] Found valid transfrom")
                self.broadcast(calibration_transform)
                self.client.save()
                return FlexGraspErrorCodes.SUCCESS
            else:
                rospy.logwarn("[CALIBRATE] Computed calibration is invalid")
                return FlexGraspErrorCodes.FAILURE

    def broadcast(self, result):
        rospy.loginfo("Broadcasting result")
        broadcaster = tf2_ros.StaticTransformBroadcaster()

        static_transformStamped = result.calibration.transform
        broadcaster.sendTransform(static_transformStamped)

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        if (self.command == 'e_init'):
            result = FlexGraspErrorCodes.SUCCESS

        elif (self.command == 'calibrate'):
            result = self.init_poses_1()

            if result == FlexGraspErrorCodes.SUCCESS:
                result = self.calibrate()

        elif (self.command == 'calibrate_height'):
            result = self.init_poses_2()

            if result == FlexGraspErrorCodes.SUCCESS:
                result = self.calibrate(track_marker=False)

        elif self.command is not None:
            rospy.logwarn("[CALIBRATE] Can not take an action: received unknown command %s!", self.command)
            result = FlexGraspErrorCodes.UNKNOWN_COMMAND

        # publish success
        if result is not None:
            msg.val = result
            flex_grasp_error_log(result, self.node_name)
            self.pub_e_out.publish(msg)      
            self.command = None            


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
