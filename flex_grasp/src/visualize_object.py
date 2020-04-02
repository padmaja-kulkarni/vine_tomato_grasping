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
from flex_grasp.msg import TomatoArray

# visualivation
import moveit_commander


class VisualizeObject(object):
    def __init__(self):

        rospy.init_node("Visualize_Object",
                        anonymous=True, log_level=rospy.DEBUG)

        # init enviroment
        # moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.object_feature = None
        rospy.sleep(5)

        rospy.Subscriber("tomato", TomatoArray, self.object_feature_cb)

    def object_feature_cb(self, msg):
        if self.object_feature is None:
            self.object_feature = msg
            rospy.logdebug("Received new object feature: %s", self.object_feature)



    def add_sphere(self, timeout=4, box_name = ''):
        """ create a box with a given name.

        Args:

        Returns: True if the box was succesfully added, False otherwise.

        """
        box_pose = PoseStamped()
        i = 0
        succes = True
        rospy.logdebug("Tomato 1: %s", self.object_feature.tomatoes[0])
        rospy.logdebug("Tomato 2: %s", self.object_feature.tomatoes[1])

        for tomato in self.object_feature.tomatoes:
            box_pose.header.frame_id =  tomato.header.frame_id
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position = tomato.position
            radius = tomato.radius
            current_sphere_name = box_name + "_" + str(i)

            rospy.logdebug("Adding tomato: %s", current_sphere_name)
            # Add box
            self.scene.add_sphere(current_sphere_name, box_pose, radius = radius)
            i = i + 1

            # Check if box has been added
            if not self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name = current_sphere_name):
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


    def visualize(self):
        if self.object_feature is not None:
            if self.add_sphere(box_name = 'tomato'):
                rospy.logdebug("added tomato")
                self.object_feature = None

def main():
    try:
        visualize_object = VisualizeObject()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            visualize_object.visualize()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
