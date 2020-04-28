#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:49:29 2020

@author: jelle
"""

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from flex_grasp.msg import Truss
from std_msgs.msg       import Float64

from func.conversions import pose_to_lists
from func.utils import add_lists, multiply_lists
from moveit_commander.conversions import list_to_pose
from math import pi

import tf2_ros
import tf
import tf2_geometry_msgs

class PoseTransform(object):
    """PoseTransform"""
    def __init__(self):
        self.event = None
        self.object_features = None

        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)

        # Initialize Subscribers
        rospy.Subscriber("object_features", Truss, self.object_features_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Initialize Publishers
        self.pub_pre_grasp_pose = rospy.Publisher('preGraspPose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_grasp_pose = rospy.Publisher('graspPose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_pre_place_pose = rospy.Publisher('prePlacePose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_place_pose = rospy.Publisher('placePose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_ee_distance = rospy.Publisher('endEffectorDistance',
                                    Float64, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        self.use_sdh = rospy.get_param('use_sdh')

        if self.use_iiwa:
            self.grasp_position_transform = [0, 0, -0.06] # [m]
            self.pre_grasp_position_transform = [0, 0, 0.05] # [m]
            self.orientation_transform = [0, 0, -pi/2]
        if self.use_interbotix:
            self.grasp_position_transform =     [0, 0, 0.04] # [m]
            self.pre_grasp_position_transform = [0, 0, 0.10] # [m]
            self.orientation_transform = [-pi, pi/2, 0]


        self.place_orientation_transform = [1.0, 1.0, -1.0]
        self.place_position_transform = [0.0, 0.0, 0.0]

        if self.use_interbotix:

            self.pre_grasp_ee = {
                "left_finger"   :0.03,
                "right_finger"  :-0.03}

            self.grasp_ee = {
                "left_finger"   :0.015,
                "right_finger"  :-0.015}
        if self.use_iiwa:

            self.pre_grasp_ee = {
                "sdh_finger_12_joint"   :-0.7,
                "sdh_finger_13_joint"   :1.57,
                "sdh_finger_21_joint"   :0,
                "sdh_finger_22_joint"   :-0.7,
                "sdh_finger_23_joint"   :1.57,
                "sdh_knuckle_joint"     :0,
                "sdh_thumb_2_joint"     :-0.7,
                "sdh_thumb_3_joint"     : 1.57}

            self.grasp_ee = {
                "sdh_finger_12_joint"   :0,
                "sdh_finger_13_joint"   :1.57,
                "sdh_finger_21_joint"   :0,
                "sdh_finger_22_joint"   :0,
                "sdh_finger_23_joint"   :1.57,
                "sdh_knuckle_joint"     :0,
                "sdh_thumb_2_joint"     :0,
                "sdh_thumb_3_joint"     :1.57}

        # Listen
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #wait until transform is obtained
        self.trans = None

        while self.trans is None:
            self.get_trans()
            rospy.sleep(0.1)


    def object_features_cb(self, msg):
        if self.object_features is None:
            self.object_features = msg
            rospy.logdebug("[POSE TRANSFORM] Received new object feature message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = "e_start"
            rospy.logdebug("[POSE TRANSFORM] Received new pose transform event message")

            # self.pub_e_out.publish(msg_e)
            # msg_e = String()
            # msg_e.data = "e_wait"

    def get_trans(self):
        if not (self.object_features is None):
            try:
                self.trans = self.tfBuffer.lookup_transform('world',self.object_features.cage_location.header.frame_id,  rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            # continue

    def transform_pose(self):
        if self.event == "e_start":
            if self.object_features is None:
                rospy.logwarn("[POSE TRANSFORM] Cannot transform pose, since it is still empty!")
            else:
                msg_e = String()
                msg_e.data = "e_success"

                self.object_pose = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, self.trans)
                end_effector_distance = 0.3*2*self.object_features.peduncle.radius

                pre_grasp_pose = self.object_pose_to_grasp_pose(self.pre_grasp_position_transform)
                grasp_pose = self.object_pose_to_grasp_pose(self.grasp_position_transform)
                pre_place_pose = self.object_pose_to_place_pose(pre_grasp_pose)
                place_pose = self.object_pose_to_place_pose(grasp_pose)

                self.pub_pre_grasp_pose.publish(pre_grasp_pose)
                self.pub_grasp_pose.publish(grasp_pose)
                self.pub_pre_place_pose.publish(pre_place_pose)
                self.pub_place_pose.publish(place_pose)

                rospy.set_param('pre_grasp_ee', self.pre_grasp_ee)
                rospy.set_param('grasp_ee', self.grasp_ee)

                self.pub_ee_distance.publish(end_effector_distance)
                self.pub_e_out.publish(msg_e)

                self.object_features = None
                self.event = None

    def object_pose_to_grasp_pose(self, position_transform):

        object_pose = self.object_pose
        grasp_pose = PoseStamped()
        grasp_pose.header = object_pose.header

        # position
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        grasp_position = add_lists(object_position, position_transform)
        grasp_orientation = add_lists(object_orientation, self.orientation_transform)

        grasp_pose.pose = list_to_pose(grasp_position + grasp_orientation)

        return grasp_pose

    def object_pose_to_place_pose(self, grasp_pose):

        place_pose = PoseStamped()
        place_pose.header = grasp_pose.header

        # position
        grasp_position, grasp_orientation = pose_to_lists(grasp_pose.pose, 'euler')
        place_position = add_lists(grasp_position, self.place_position_transform)
        place_orientation = multiply_lists(grasp_orientation, self.place_orientation_transform)

        place_pose.pose = list_to_pose(place_position + place_orientation)

        return place_pose
        # return self.object_pose_to_grasp_pose(height)

def main():
    try:
        pose_transform = PoseTransform()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            pose_transform.transform_pose()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
