#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 10:46:14 2020

@author: jelle
"""

import rospy
from std_msgs.msg import String


class PipeLine(object):
    """PipeLine"""
    def __init__(self):

        # Initialize Variables
        self.state = "IDLE"
        self.state_previous = "IDLE"
        self.command = None

        self.object_detected = None
        self.pose_transformed = None
        self.robot_moved = None

        rospy.init_node("Pipe_Line",
			anonymous=True, log_level=rospy.DEBUG)

        ## Initialize Publishers

        self.pub_obj_detection = rospy.Publisher("Object_Detection/e_in",
                                      String, queue_size=10, latch=True)

        self.pub_pose_transform = rospy.Publisher("Pose_Transform/e_in",
                                      String, queue_size=10, latch=True)

        self.pub_move_robot = rospy.Publisher("Move_Robot/e_in",
                                      String, queue_size=10, latch=True)

        ## Initialize Subscribers
        rospy.Subscriber("pipelineState", String, self.pipeline_state_call_back)
        rospy.Subscriber("Object_Detection/e_out", String, self.object_detection_call_back)
        rospy.Subscriber("Pose_Transform/e_out", String, self.pose_transform_call_back)
        rospy.Subscriber("Move_Robot/e_out", String, self.move_robot_call_back)

    ### Callback Functions
    ## Pipeline State Callback Function
    def pipeline_state_call_back(self, msg):
        if self.command == None:
            self.command = msg.data
            rospy.logdebug("[PIPELINE] Received new pipeline goal state event message: %s", self.command)

    ## Object Detection Callback Function
    def object_detection_call_back(self, msg):
        if self.object_detected == None:
            self.object_detected = msg.data
            rospy.logdebug("[PIPELINE] Received new object detected event message %s", self.object_detected)

    ## Pose Transform Callback Function
    def pose_transform_call_back(self, msg):
        if self.pose_transformed == None:
            self.pose_transformed = msg.data
            rospy.logdebug("[PIPELINE] Received new pose transformed event message: %s", self.pose_transformed)

    def move_robot_call_back(self, msg):
        if self.robot_moved == None:
            rospy.logdebug("[PIPELINE] Received new move robot event out message: %s ", msg.data)
            if msg.data == "e_success":
                self.robot_moved = True
            elif msg.data == "e_failure":
                self.robot_moved = False


    ### Log state update
    def update_state(self, new_state):
        self.state_previous = self.state
        self.state = new_state

        rospy.loginfo("[PIPELINE] updated pipeline state, from %s to %s",
                      self.state_previous, self.state)

    ### Run Function

    def set_state(self):

        ## update current state
        if (self.state == "IDLE") and (self.command == "OPEN"):
            self.update_state("OPEN")
            self.command = None
            self.send_message()

        if (self.state == "IDLE") and (self.command == "CLOSE"):
            self.update_state("CLOSE")
            self.command = None
            self.send_message()

        if (self.state == "IDLE") and (self.command == "HOME"):
            self.update_state("HOME")
            self.command = None
            self.send_message()

        if (self.state == "IDLE") and ((self.command == "MOVE") or (self.command == "PICK")):
            self.update_state("DETECT")
            self.send_message()

        if self.object_detected and self.state == "DETECT":
            self.object_detected = None
            self.update_state("TRANSFORM")
            self.send_message()

        if self.pose_transformed and self.state == "TRANSFORM":
            self.update_state(self.command)
            self.command = None
            self.pose_transformed = None
            self.send_message()

        if (self.state == "PICK") and (self.command == "PLACE"):
            self.robot_moved = None
            self.command = None
            self.update_state("PLACE")
            self.send_message()

        if self.robot_moved and (self.state == "PICK" or self.state == "PLACE" or self.state == "HOME" or self.state == "MOVE" or self.state == "OPEN"  or self.state == "CLOSE"):
            self.robot_moved = None
            self.update_state("IDLE")
            self.send_message()

        if (self.robot_moved == False) and (self.state == "PICK" or self.state == "PLACE" or self.state == "HOME" or self.state == "MOVE"  or self.state == "OPEN"  or self.state == "CLOSE"):
            rospy.logwarn("Robot did not move!")
            self.robot_moved = None
            self.update_state("IDLE")
            self.send_message()

    def send_message(self):
    ## command other nodes
        if self.state == "DETECT":
            self.start_obj_detection()

        if self.state == "TRANSFORM":
            self.send_start_to_pose_transform()

        if self.state == "MOVE":
            self.send_move_to_move_robot()

        if self.state == "OPEN":
            self.send_open_to_move_robot()

        if self.state == "CLOSE":
            self.send_close_to_move_robot()

        if self.state == "PICK":
            self.send_pick_to_move_robot()

        if self.state == "PLACE":
            self.send_place_to_move_robot()

        if self.state == "HOME":
            self.send_home_to_move_robot()

    ### Send Start Functions
    def start_obj_detection(self):
        self.pub_obj_detection.publish("e_start")

    def stop_obj_detection(self):
        self.pub_obj_detection.publish("e_stop")

    def send_start_to_pose_transform(self):
        self.pub_pose_transform.publish("e_start")

    def send_move_to_move_robot(self):
        self.pub_move_robot.publish("move")

    def send_pick_to_move_robot(self):
        self.pub_move_robot.publish("pick")

    def send_place_to_move_robot(self):
        self.pub_move_robot.publish("place")

    def send_home_to_move_robot(self):
        self.pub_move_robot.publish("home")

    def send_open_to_move_robot(self):
        self.pub_move_robot.publish("open")

    def send_close_to_move_robot(self):
        self.pub_move_robot.publish("close")


def main():
    try:
        pipeline = PipeLine()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            pipeline.set_state()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()