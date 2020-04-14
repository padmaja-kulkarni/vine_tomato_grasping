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

        rospy.init_node("Pose_Transform",
                        anonymous=True, log_level=rospy.DEBUG)

        # Initialize Subscribers
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Initialize Publishers
        self.pub_pose = rospy.Publisher('endEffectorPose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        self.use_sdh = rospy.get_param('use_sdh')


    def object_features_cb(self, msg):
        if self.object_features is None:
            self.object_features = msg
            rospy.logdebug("Received new object features message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = "e_start"
            rospy.logdebug("[Pose Transform] Received new event message: %s", self.event)


    def transform_pose(self):
        if self.event == "e_start":
            msg_e = String()
            msg_e.data = "e_success"

            self.end_effector_pose = self.generate_pose()

            self.pub_pose.publish(self.end_effector_pose)
            self.pub_e_out.publish(msg_e)

            self.event = None

    def generate_pose(self):

        end_effector_pose = PoseStamped()

        end_effector_pose.header.frame_id = "world"
        end_effector_pose.header.stamp = rospy.Time.now()

        # position
        end_effector_pose.pose.position.x = 0.15
        end_effector_pose.pose.position.y = 0
        if self.use_iiwa:
            end_effector_pose.pose.position.z = 0.1# + 0.15
        elif self.use_interbotix:
            end_effector_pose.pose.position.z = 0.07

        # orientation
        euler = [3.1415/2, 0, 0]

        if self.use_iiwa:
            quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] - 3.1415/2) # move parralel to object
        elif self.use_interbotix:
            quat = tf.transformations.quaternion_from_euler(euler[0]- 3.1415, euler[1] + 3.1415/2, euler[2])
        rospy.logdebug("Quaternion desired: %s", quat)

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
