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
        self.pub_ee_pose = rospy.Publisher('endEffectorPose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_ee_distance = rospy.Publisher('endEffectorDistance',
                                    Float64, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        self.use_sdh = rospy.get_param('use_sdh')

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
        if not (self.object_features is None):
            try:
                self.trans = self.tfBuffer.lookup_transform('world',self.object_features.cage_location.header.frame_id,  rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            # continue

    def transform_pose(self):
        if self.event == "e_start":
            if self.object_features is None:
                rospy.logwarn("Cannot transform pose, since it is still empty!")
            else:
                msg_e = String()
                msg_e.data = "e_success"

                self.object_pose = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, self.trans)
                self.end_effector_distance = 0.3*2*self.object_features.peduncle.radius

                self.end_effector_pose = self.object_pose_to_end_effector_pose(self.object_pose)

                self.pub_ee_pose.publish(self.end_effector_pose)
                self.pub_ee_distance.publish(self.end_effector_distance)
                self.pub_e_out.publish(msg_e)

                self.object_features = None
                self.event = None

    def object_pose_to_end_effector_pose(self, object_pose):

        end_effector_pose = PoseStamped()

        end_effector_pose.header = object_pose.header

        # position
        position = object_pose.pose.position
        end_effector_pose.pose.position.x = position.x
        end_effector_pose.pose.position.y = position.y
        if self.use_iiwa:
            end_effector_pose.pose.position.z = position.z# + 0.15
        elif self.use_interbotix:
            end_effector_pose.pose.position.z = position.z + 0.04

        # orientation
        orientation = object_pose.pose.orientation
        rotation = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)

        if self.use_iiwa:
            quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] - 3.1415/2) # move parralel to object
        elif self.use_interbotix:
            quat = tf.transformations.quaternion_from_euler(euler[0]- 3.1415, euler[1] + 3.1415/2, euler[2])

        end_effector_pose.pose.orientation.x = quat[0]
        end_effector_pose.pose.orientation.y = quat[1]
        end_effector_pose.pose.orientation.z = quat[2]
        end_effector_pose.pose.orientation.w = quat[3]

        return end_effector_pose

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
