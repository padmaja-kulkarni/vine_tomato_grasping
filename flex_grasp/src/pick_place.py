#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu May 28 09:35:43 2020

@author: taeke
"""

import rospy

# messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from func.ros_utils import wait_for_success

class PickPlace(object):
    
    def __init__(self):   
        
        
        self.state = "init"
        self.prev_state = None
        self.command = None

        self.pre_grasp_pose = None
        self.grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None        
        
        
        self.object_features = None
        
        self.debug_mode = rospy.get_param("pick_place/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[POSE TRANSFORM] Launching object detection node in debug mode")
        else:
            log_level = rospy.INFO
        
        rospy.init_node("pick_place", anonymous=True, log_level=log_level)
                        
        self.pub_e_out = rospy.Publisher("~e_out", String, queue_size=10, latch=True)
                                             
        self.pub_move_robot_command = rospy.Publisher("move_robot/e_in", String, queue_size=10, latch=False)
        
        self.pub_move_robot_pose = rospy.Publisher("robot_pose", PoseStamped, queue_size=10, latch=False)
                                     
         # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        rospy.Subscriber("pre_grasp_pose", PoseStamped, self.pre_grasp_pose_cb)
        rospy.Subscriber("grasp_pose", PoseStamped, self.grasp_pose_cb)
        rospy.Subscriber("pre_place_pose", PoseStamped, self.pre_place_pose_cb)
        rospy.Subscriber("place_pose", PoseStamped, self.place_pose_cb)
    
    
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[PICK PLACE] Received command in message: %s", self.command)

            # reset outputting message
            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)    
    
    
    def grasp_pose_cb(self, msg):
        if self.grasp_pose is None:
            self.grasp_pose = msg
            rospy.logdebug("[PICK PLACE] Received new grasp pose massage")
            # self.load_param(1.0)

    def pre_grasp_pose_cb(self, msg):
        if self.pre_grasp_pose is None:
            self.pre_grasp_pose = msg
            rospy.logdebug("[PICK PLACE] Received new pre grasp pose massage")

    def pre_place_pose_cb(self, msg):
        if self.pre_place_pose is None:
            self.pre_place_pose = msg
            rospy.logdebug("[PICK PLACE] Received new pre place pose message")

    def place_pose_cb(self, msg):
        if self.place_pose is None:
            self.place_pose = msg
            rospy.logdebug("[PICK PLACE] Received new placing pose message")    
    
    
    def command_to_pose(self, pose):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pose")
        self.pub_move_robot_pose.publish(pose)
        self.pub_move_robot_command.publish("move_manipulator")
        success = wait_for_success("move_robot/e_out", 5)
        return success
        
    def command_to_home(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to home")
        self.pub_move_robot_command.publish("home")
        success = wait_for_success("move_robot/e_out", 5) 
        return success
                

    def apply_pre_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying pre-grasp with end effector")
        self.pub_move_robot_command.publish("open")
        success = wait_for_success("move_robot/e_out", 5)        
        return success
                      
                      
    def apply_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying grasp with end effector")
        self.pub_move_robot_command.publish("grasp")
        success = wait_for_success("move_robot/e_out", 5)        
        return success
        
        
    def apply_release_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying release with end effector")
        self.pub_move_robot_command.publish("ee_release")
        success = wait_for_success("move_robot/e_out", 5)        
        return success
        
                      
    def pick(self):
        rospy.logdebug("[PICK PLACE] Picking object")

        success =  self.command_to_pose(self.pre_grasp_pose)

        if success:
            success = self.apply_pre_grasp_ee()

        if success:
            success = self.command_to_pose(self.grasp_pose)

        if success:
            success = self.apply_grasp_ee()

        if success:
            success = self.command_to_pose(self.pre_grasp_pose)

        return success


    def place(self):
        rospy.logdebug("[PICK PLACE] Placing object")
        success = self.command_to_pose(self.pre_place_pose)

        if success:
            success = self.command_to_pose(self.place_pose)

        if success:
            success = self.apply_release_ee()

        if success:
            success = self.command_to_pose(self.pre_place_pose)

        if success:
            success = self.command_to_home()

        self.reset_msg()
        return success
            
            
    def reset_msg(self):
        rospy.logdebug("[PICK PLACE] Resetting for next grasp")
        self.pre_grasp_ee = None
        self.grasp_ee = None
        self.grasp_pose = None
        self.pre_grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None
        return True            
            

    def received_all_data(self):
        success =(self.grasp_pose != None) and (self.pre_grasp_pose != None) and (self.pre_place_pose != None) and (self.place_pose != None)
        # received_all_data        
        return success

            
    ### Log state update
    def log_state_update(self):
        rospy.loginfo("[PICK PLACE] updated move robot state, from %s to %s",
                      self.prev_state, self.state)            
            
                  
    def update_state(self, success):

        if (self.state == "idle") and not self.received_all_data():
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()

        elif self.state == "init" and self.received_all_data():
            self.prev_state = self.state
            self.state = "idle"
            self.log_state_update()

        elif (self.state == "idle") and ((self.command == "pick") or (self.command == "pick_place")) and success:
            self.prev_state = self.state
            self.state = "picked"
            self.log_state_update()

        elif (self.state == "picked") and success:
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()                  
                  
                        
    def take_action(self):

        success = None
        msg = String()

        # State dependent actions
        if self.state == "init":
            if self.command == "pick" or self.command == "pick_place":
                rospy.logwarn("[PICK PLACE] Cannot pick object, it still needs to be detected!")
                # rospy.sleep(3.0)
                success = False

        if self.command == "e_init":
            rospy.logdebug("[PICK PLACE] executing e_init command")
            success = True
            
        if self.state == "idle":
            if self.command == "pick" or self.command == "pick_place":
                success = self.pick()

        elif self.state == "picked":
            if self.command == "place" or self.command == "pick_place":
                success = self.place()
            
        elif self.command == "reset":
            success = self.reset_msg()

        self.update_state(success)

        if self.command == "pick_place" and self.state == "picked":
            success = None

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.command = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Pick place failed to execute command: %s", self.command)
                self.command = None

            self.pub_e_out.publish(msg)                      
                      
                      
def main():
    try:
        pick_place = PickPlace()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            pick_place.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
