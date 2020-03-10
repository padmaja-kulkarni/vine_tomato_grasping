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
        self.state_goal = None
        
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
        
        self.pscb_lambda = lambda msg: self.pscb(msg)
        rospy.Subscriber("pipelineState", 
                         String, self.pscb_lambda)
        
        self.odcb_lambda = lambda msg: self.odcb(msg)
        rospy.Subscriber("Object_Detection/e_out",
                         String, self.odcb_lambda)
        
        self.ptcb_lambda = lambda msg: self.ptcb(msg)
        rospy.Subscriber("Pose_Transform/e_out", 
                         String, self.ptcb_lambda)
        
        self.mrcb_lambda = lambda msg: self.mrcb(msg)
        rospy.Subscriber("Move_Robot/e_out", 
                         String, self.mrcb_lambda)
     
        ### Callback Functions
        
        ## Pipeline State Callback Function
        
    def pscb(self, msg):
        if self.state_goal == None:
            self.state_goal = msg.data
            rospy.logdebug("Received new pipeline state message")
            
    ## Object Detection Callback Function
        
    def odcb(self, msg):
        if self.object_detected == None:
            self.object_detected = msg.data
            rospy.logdebug("Received new object detected message")
    
    ## Pose Transform Callback Function
    
    def ptcb(self, msg):
        if self.pose_transformed == None:
            self.pose_transformed = msg.data
            rospy.logdebug("Received new pose transformed message")
    
    ## Move Robot Callback Function
    
    def mrcb(self, msg):
        if self.robot_moved == None:
            self.robot_moved = msg.data
            rospy.logdebug("Received new robot moved message")
    
    ### Log state update
    def log_state_update(self):
        rospy.loginfo("updated pipeline state, from %s to %s", 
                      self.state_previous, self.state)

    ### Run Function
            
    def run(self):
        
        ## update current state
        if (self.state == "IDLE") and (self.state_goal == "DETECT"):
            self.state_previous = self.state
            self.state = "DETECT"
            self.state_goal = None
            self.log_state_update()
            rospy.loginfo("Updated state")
            
        if self.object_detected and self.state == "DETECT":
            self.state_previous = self.state
            self.state = "TRANSFORM"
            self.object_detected = None
            self.log_state_update()
            
        if self.pose_transformed and self.state == "TRANSFORM":
            self.state_previous = self.state
            self.state = "MOVE"
            self.pose_transformed = None
            self.log_state_update()
            
        if self.robot_moved and self.state == "MOVE":
            self.state_previous = self.state
            self.state = "IDLE"
            self.robot_moved = None
            self.log_state_update()
            
        ## command other nodes
        if self.state == "DETECT":
            self.start_obj_detection()
        
        if self.state == "TRANSFORM":
            self.send_start_to_pose_transform()
            
        if self.state == "MOVE":
            self.send_start_to_move_robot()
            
    ### Send Start Functions
            
    def start_obj_detection(self):
        self.pub_obj_detection.publish("e_start")
        
        
    def stop_obj_detection(self):
        self.pub_obj_detection.publish("e_stop")
        
        
    def send_start_to_pose_transform(self):
        self.pub_pose_transform.publish("e_start")
        
    def send_start_to_move_robot(self):
        self.pub_move_robot.publish("e_start")
        
        
def main():
    try:
        PL = PipeLine()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            PL.run()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == '__main__':
    main()
            
    
