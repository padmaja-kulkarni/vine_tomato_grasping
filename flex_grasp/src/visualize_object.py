#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:30:31 2020

@author: taeke
"""


import sys
import rospy

# msg
from geometry_msgs.msg import PoseStamped
from flex_grasp.msg import Tomato
from flex_grasp.msg import Truss
from sensor_msgs.msg import PointCloud2

# visualivation
import moveit_commander


class VisualizeObject(object):
    def __init__(self):

        self.debug_mode = rospy.get_param("visualize_object/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[VISUALIZE OBJECT] Luanching visualize object in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("visualize_object",
                        anonymous=True, log_level=log_level)

        # init enviroment
        # moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.object_feature = None
        rospy.sleep(5)

        rospy.Subscriber("object_features", Truss, self.object_feature_cb)

    def object_feature_cb(self, msg):
        if self.object_feature is None:
            self.object_feature = msg
            rospy.logdebug("[VISUALIZE OBJECT] Received new object feature message")



    def add_peduncle(self, timeout=4, name = 'peduncle'):
        pose = PoseStamped()
        pose = self.object_feature.peduncle.pose
        length = self.object_feature.peduncle.length
        radius = self.object_feature.peduncle.radius
        size = (length, 2*radius, 2*radius)

        self.scene.add_box(name, pose, size)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name = name)

    def add_tomatoes(self, timeout=4, name = 'tomato'):
        """ create a box with a given name.

        Args:

        Returns: True if the box was succesfully added, False otherwise.

        """
        box_pose = PoseStamped()
        i = 0
        succes = True

        for tomato in self.object_feature.tomatoes:
            box_pose.header.frame_id =  tomato.header.frame_id
            rospy.logdebug("[Visualize Object] frame ID: %s", tomato.header.frame_id)
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position = tomato.position
            radius = tomato.radius
            current_name = name + "_" + str(i)

            self.scene.add_sphere(current_name, box_pose, radius = radius)
            i = i + 1

            # Check if box has been added
            if not self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name = current_name):
                succes = False


        return succes

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name= ''):
        """ Wait until we see the changes reflected in the scene

            Args:

            Returns: True if the box was succesfully added, False otherwise.
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects()
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            known_objects = self.scene.get_known_object_names()
            rospy.logdebug("Known objects: %s", known_objects)

            is_known = box_name in known_objects

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def visualize_truss(self):

        # Add tomatoes
        if not self.add_tomatoes():
            rospy.logwarn("Failed to add tomatoes")


        # Add peduncle
        if not self.add_peduncle():
            rospy.logwarn("Failed to add peduncle")

        self.object_feature = None

def main():
    try:
        visualize_object = VisualizeObject()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            if visualize_object.object_feature is not None:
                visualize_object.visualize_truss()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
