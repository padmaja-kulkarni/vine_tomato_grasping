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
        self.pub_pose = rospy.Publisher('~endEffectorPose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)


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
            rospy.logdebug("Received new object features message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = "e_start"
            rospy.logdebug("Received new move robot event message")

    def get_trans(self):
        try:
            self.trans = self.tfBuffer.lookup_transform('world','camera_color_optical_frame',  rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            # continue


    def object_pose_to_end_effector_pose(self):
        self.end_effector_pose = self.object_pose_transformed
        self.end_effector_pose.pose.position.z = self.end_effector_pose.pose.position.z + 0.15


    def transform_pose(self):
        if self.event == "e_start":
            if self.object_features is None:
                rospy.logwarn("Cannot transform pose, since it is still empty!")
            else:
                msg_e = String()
                msg_e.data = "e_success"

                self.object_pose_transformed = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, self.trans)

                self.object_pose_to_end_effector_pose()

                self.pub_pose.publish(self.end_effector_pose)
                self.pub_e_out.publish(msg_e)

                self.object_features = None
                self.event = None


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
