#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 21:07:08 2020

@author: taeke
"""
import rospy
import tf
import math

from flex_grasp.msg import Tomato
from flex_grasp.msg import Truss
from flex_grasp.msg import Peduncle
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class ObjectDetection(object):
    """ObjectDetection"""
    def __init__(self):

        self.event = None

        rospy.init_node("Object_Detection",
                        anonymous=True, log_level=rospy.DEBUG)

        # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publish
        self.pub_e_out = rospy.Publisher("~e_out",
                                         String, queue_size=10, latch=True)

        self.pub_object_features = rospy.Publisher("object_features",
                                        Truss, queue_size=5, latch=True)

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("Received new move robot event message: %s", self.event)


    def detect_object(self):
        if self.event == "e_start":

            #%%##################
            ### Cage location ###
            #####################
            point = [0.15, 0, 0.05]
            angle = 1 #3.1415/2.0
            frame = "world"
            cage_pose =  point_to_pose_stamped(point, angle, frame)

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
            t1x = point[0] + (L/2 + radii[0])*math.cos(angle)
            t1y = point[1] - (L/2 + radii[0])*math.sin(angle)
            t2x = point[0] - (L/2 + radii[1])*math.cos(angle)
            t2y = point[1] + (L/2 + radii[1])*math.sin(angle)
            point1 = [t1x, t1y, 0]
            point2 = [t2x, t2y, 0]
            points = [point1, point2]

            tomatoes = []
            for point, radius in zip(points, radii):
                tomatoes.append(point_to_tomato(point, radius, frame))

            #%%##########
            ### Truss ###
            #############

            truss = Truss()
            truss.tomatoes = tomatoes
            truss.cage_location = cage_pose
            truss.peduncle = peduncle

            msg_e = String()
            msg_e.data = "e_success"

            self.event = None
            rospy.logdebug(truss)
            self.pub_object_features.publish(truss)
            self.pub_e_out.publish(msg_e)

def point_to_pose_stamped(point, angle, frame):

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame
    pose_stamped.header.stamp = rospy.Time.now()

    quat = tf.transformations.quaternion_from_euler(0, 0, -angle)
    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]
    pose_stamped.pose.position.x = point[0]
    pose_stamped.pose.position.y = point[1]
    pose_stamped.pose.position.z = point[2]

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
