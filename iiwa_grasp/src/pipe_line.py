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
        self.object_detected = None
        self.pose_transformed = None
        self.robot_moved = None
        
        rospy.init_node("Pipe_Line",
			anonymous=True, log_level=rospy.DEBUG)
        
        ## Initialize Publishers
        
        self.pub_od = rospy.Publisher("Object_Detection/e_in", 
                                      String, queue_size=10, latch=True)
        
        self.pub_pt = rospy.Publisher("Pose_Transform/e_in",
                                      String, queue_size=10, latch=True)
        
        self.pub_mr = rospy.Publisher("Move_Robot/e_in", 
                                      String, queue_size=10, latch=True)
        
        ## Initialize Subscribers
        
        self.pscb_lambda = lambda msg: self.pscb(msg, self)
        rospy.Subscriber("pipelineState", 
                         String, self.pscb_lambda)
        
        self.odcb_lambda = lambda msg: self.odcb(msg, self)
        rospy.Subscriber("Object_Detection/e_out",
                         String, self.odcb_lambda)
        
        self.ptcb_lambda = lambda msg: self.ptcb(msg, self)
        rospy.Subscriber("Pose_Transform/e_out", 
                         String, self.ptcb_lambda)
        
        self.mrcb_lambda = lambda msg: self.mrcb(msg, self)
        rospy.Subscriber("Move_Robot/e_out", 
                         String, self.mrcb_lambda)
     
        
    ### Callback Functions
        
    ## Pipeline State Callback Function
        
    def pscb(msg, self):
        if self.state == None:
            self.state = msg.data
            rospy.logdebug("Received new pipeline state message")
            
    ## Object Detection Callback Function
        
    def odcb(msg, self):
        if self.object_detected == None:
            self.object_detected = msg.data
            rospy.logdebug("Received new object detected message")
    
    ## Pose Transform Callback Function
    
    def ptcb(msg, self):
        if self.pose_transformed == None:
            self.pose_transformed = msg.data
            rospy.logdebug("Received new pose transformed message")
    
    ## Move Robot Callback Function
    
    def mrcb(msg, self):
        if self.robot_moved == None:
            self.robot_moved = msg.data
            rospy.logdebug("Received new robot moved message")
    

    ### Run Function
            
    def run(self):
        if self.state == "IDLE":
            pass
        
        if self.state == "DETECT":
            self.send_start_to_obj_detection()
            
        if self.object_detected and self.state == "DETECT":
            self.state = "TRANSFORM"
            self.object_detected = None
            
        if self.state == "TRANSFORM":
            self.send_start_to_pose_transform()
            
        if self.pose_transformed and self.state == "TRANSFORM":
            self.state = "MOVE"
            self.pose_transformed = None
            
        if self.state == "MOVE":
            self.send_start_to_move_robot()
            
        if self.robot_moved and self.state == "MOVE":
            self.state = "IDLE"
            self.robot_moved = None
            
    ### Send Start Functions
            
    def send_start_to_obj_detection(self):
        self.pub_od.publish("e_start")
        
    def send_start_to_pose_transform(self):
        self.pub_pt.publish("e_start")
        
    def send_start_to_move_robot(self):
        self.pub.mr.publish("e_start")
        
        
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
            
    
