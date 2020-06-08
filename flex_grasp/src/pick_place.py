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
from flex_grasp.msg import Truss

from func.ros_utils import wait_for_success
from func.ros_utils import wait_for_variable

from func.conversions import pose_to_lists
from moveit_commander.conversions import list_to_pose
from func.utils import add_lists, multiply_lists

import tf2_ros
import tf2_geometry_msgs

from math import pi

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
            rospy.loginfo("[PICK PLACE] Launching pick place node in debug mode")
        else:
            log_level = rospy.INFO
        
        rospy.init_node("pick_place", anonymous=True, log_level=log_level)
                        
        self.pub_e_out = rospy.Publisher("~e_out", String, queue_size=10, latch=True)
                                             
        self.pub_move_robot_command = rospy.Publisher("move_robot/e_in", String, queue_size=10, latch=False)
        
        self.pub_move_robot_pose = rospy.Publisher("robot_pose", PoseStamped, queue_size=10, latch=False)
        
        # Initialize Publishers
        self.pub_pre_grasp_pose = rospy.Publisher('pre_grasp_pose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_grasp_pose = rospy.Publisher('grasp_pose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_pre_place_pose = rospy.Publisher('pre_place_pose',
                                        PoseStamped, queue_size=5, latch=True)

        self.pub_place_pose = rospy.Publisher('place_pose',
                                        PoseStamped, queue_size=5, latch=True)

                                     
         # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("object_features", Truss, self.object_features_cb)

        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        self.use_sdh = rospy.get_param('use_sdh')

        if self.use_iiwa:
            rospy.logwarn("No pose trnaform for iiwa available...")
        if self.use_interbotix:
            self.grasp_position_transform =     [0, 0, 0.04] # [m]
            self.pre_grasp_position_transform = [0, 0, 0.10] # [m]
            self.orientation_transform = [-pi, pi/2, 0]


        self.place_orientation_transform = [1.0, 1.0, -1.0]
        self.place_position_transform = [0.0, 0.0, 0.0]


        # Tranform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None
    
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[PICK PLACE] Received command in message: %s", self.command)

            # reset outputting message
            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)    
    
    def object_features_cb(self, msg):
        if self.object_features is None:
            self.object_features = msg
            rospy.logdebug("[PICK PLACE] Received new object feature message")    
    
    
    def get_trans(self):
        if not (self.object_features is None):
            try:
                self.trans = self.tfBuffer.lookup_transform('world',self.object_features.cage_location.header.frame_id,  rospy.Time(0))
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False

    def transform_pose(self):
        if not wait_for_variable(3, self.object_features):
            rospy.logwarn("[PICK PLACE] Cannot transform pose, since object_features still empty!")
            return False
            
        if not self.get_trans():
            rospy.logwarn("[PICK PLACE] Cannot transform pose, failed to lookup transform!!")
            return False

        self.object_pose = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, self.trans)

        self.pre_grasp_pose = self.object_pose_to_grasp_pose(self.pre_grasp_position_transform)
        self.grasp_pose = self.object_pose_to_grasp_pose(self.grasp_position_transform)
        self.pre_place_pose = self.object_pose_to_place_pose(self.pre_grasp_pose)
        self.place_pose = self.object_pose_to_place_pose(self.grasp_pose)
        
        self.pub_all_pose()
        
        # reset
        self.object_features = None
        return True

    def pub_all_pose(self):
        self.pub_pre_grasp_pose.publish(self.pre_grasp_pose)
        self.pub_grasp_pose.publish(self.grasp_pose)
        self.pub_pre_place_pose.publish(self.pre_place_pose)
        self.pub_place_pose.publish(self.place_pose)


    def object_pose_to_grasp_pose(self, position_transform):

        object_pose = self.object_pose
        grasp_pose = PoseStamped()
        grasp_pose.header = object_pose.header

        # position
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        object_orientation = (0, 0, object_orientation[2])
                
        grasp_position = add_lists(object_position, position_transform)
        grasp_orientation = add_lists(object_orientation, self.orientation_transform)

        grasp_pose.pose = list_to_pose(grasp_position + grasp_orientation)

        return grasp_pose

    def object_pose_to_place_pose(self, grasp_pose):

        place_pose = PoseStamped()
        place_pose.header = grasp_pose.header

        # position
        grasp_position, grasp_orientation = pose_to_lists(grasp_pose.pose, 'euler')
        place_position = add_lists(grasp_position, self.place_position_transform)
        place_orientation = multiply_lists(grasp_orientation, self.place_orientation_transform)

        place_pose.pose = list_to_pose(place_position + place_orientation)

        return place_pose
    
    def command_to_pose(self, pose):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pose")
        self.pub_move_robot_pose.publish(pose)
        self.pub_move_robot_command.publish("move_manipulator")
        success = wait_for_success("move_robot/e_out", 10)
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
        self.pub_move_robot_command.publish("release")
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

        elif (self.state == "idle") and (self.command == "transform") and success:
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
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be detected!")
                # rospy.sleep(3.0)
                success = False

        if self.command == "e_init":
            rospy.logdebug("[PICK PLACE] executing e_init command")
            success = True
            
        if self.command == "transform":
            rospy.logdebug("[PICK PLACE] executing transform command")
            success = self.transform_pose()
            
        if self.state == "idle":
            if self.command == "pick" or self.command == "pick_place":
                success = self.pick()

        elif self.state == "picked":
            if self.command == "place" or self.command == "pick_place":
                success = self.place()
            if self.command == "pick":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be placed!")
            
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
