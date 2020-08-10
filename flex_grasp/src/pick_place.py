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

from func.ros_utils import wait_for_result, wait_for_variable, get_transform
from func.utils import add_pose_stamped
from func.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position
from func.pick_place_result import pick_place_result_log

from flex_grasp.msg import MoveRobotResult
from flex_grasp.msg import PickPlaceResult

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
        
        self.peduncle_height = 0.075 # [m]
        self.object_features = None
        
        
        self.debug_mode = rospy.get_param("pick_place/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[PICK PLACE] Launching pick place node in debug mode")
        else:
            log_level = rospy.INFO
        
        rospy.init_node("pick_place", anonymous=True, log_level=log_level)
                        
        self.pub_e_out = rospy.Publisher("~e_out", PickPlaceResult, queue_size=10, latch=True)
                                             
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
        self.planning_frame = rospy.get_param('planning_frame')


        if self.use_iiwa:
            rospy.logwarn("No pose trnaform for iiwa available...")
        
        grasp_xyz =     [0, 0, 0.055] # [m]
        pre_grasp_xyz = [0, 0, 0.12] # [m]
        grasp_rpy = [-pi, pi/2, 0]
        place_rpy = [-pi, pi/2, 0.5]
        frame = self.planning_frame
        time = rospy.Time.now()

        self.pre_grasp_trans = point_to_pose_stamped(pre_grasp_xyz, grasp_rpy, frame, time)
        self.grasp_trans = point_to_pose_stamped(grasp_xyz, grasp_rpy, frame, time)
        self.pre_place_trans = point_to_pose_stamped(pre_grasp_xyz, place_rpy, frame, time)
        self.place_trans = point_to_pose_stamped(grasp_xyz, place_rpy, frame, time)

        # Tranform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans = None
    
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[PICK PLACE] Received command in message: %s", self.command)

            # reset outputting message
            msg = PickPlaceResult()
            msg.val = PickPlaceResult.NONE
            self.pub_e_out.publish(msg)    
    
    def object_features_cb(self, msg):
        # if self.object_features is None:
        # if self.command == "transform":
        self.object_features = msg
        rospy.logdebug("[PICK PLACE] Received new object feature message")    
  
    def get_trans(self):
        if not (self.object_features is None):
            try:
                to_frame = self.planning_frame
                from_frame = self.object_features.cage_location.header.frame_id
                if to_frame is None:
                    rospy.logwarn("Cannot find transform from cage location frame to %s, no header is defined!", to_frame)
                    return False
                    
                self.trans = self.tfBuffer.lookup_transform(to_frame, from_frame, time = rospy.Time.now())
                return True
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False

    def transform_pose(self, angle_offset = 0):
        if not wait_for_variable(3, self.object_features):
            rospy.logwarn("[PICK PLACE] Cannot transform pose, since object_features still empty!")
            return PickPlaceResult.TRANSFORM_POSE_FAILED
            
        transform = get_transform(self.planning_frame, self.object_features.cage_location.header.frame_id, self.tfBuffer)            
            
        if transform is None:
            rospy.logwarn("[PICK PLACE] Cannot transform pose, failed to lookup transform!!")
            return PickPlaceResult.TRANSFORM_POSE_FAILED

        object_pose = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, transform)
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        # object_pose.pose.position = list_to_position((object_position[0], object_position[1], self.peduncle_height))
        object_pose.pose.orientation = list_to_orientation((0, 0, object_orientation[2] + angle_offset))
        # add offsets
        self.pre_grasp_pose = add_pose_stamped(self.pre_grasp_trans, object_pose) 
        self.grasp_pose = add_pose_stamped(self.grasp_trans, object_pose) 
        self.pre_place_pose = add_pose_stamped(self.pre_place_trans, object_pose)
        self.place_pose = add_pose_stamped(self.place_trans, object_pose)
        
        self.pub_all_poses()
        return PickPlaceResult.SUCCESS

    def pub_all_poses(self):
        self.pub_pre_grasp_pose.publish(self.pre_grasp_pose)
        self.pub_grasp_pose.publish(self.grasp_pose)
        self.pub_pre_place_pose.publish(self.pre_place_pose)
        self.pub_place_pose.publish(self.place_pose)
    
    def command_to_pose(self, pose):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pose")
        if pose is None:
            rospy.logwarn("[PICK PLACE] Cannot command to pose, since the pose is None!")
            return False
        
        self.pub_move_robot_pose.publish(pose)
        self.pub_move_robot_command.publish("move_manipulator")
        return wait_for_result("move_robot/e_out", 10, MoveRobotResult)
        
    def man_pre_grasp(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pre grasp")
        
        if self.command_to_pose(self.pre_grasp_pose):
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.MAN_PRE_GRASP_FAILED
            
    def man_grasp(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to grasp")
        
        if self.command_to_pose(self.grasp_pose):
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.MAN_GRASP_FAILED
            
    def man_pre_place(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pre place")
        
        if self.command_to_pose(self.pre_place_pose):
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.MAN_PRE_PLACE_FAILED
                
    def man_place(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to place")
        
        if self.command_to_pose(self.place_pose):
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.MAN_PLACE_FAILED
        
    def command_to_home(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to home")
        self.pub_move_robot_command.publish("home")
        if wait_for_result("move_robot/e_out", 5, MoveRobotResult):
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.MAN_HOME_FAILED
                

    def apply_pre_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying pre-grasp with end effector")
        self.pub_move_robot_command.publish("open")
        if wait_for_result("move_robot/e_out", 5, MoveRobotResult): 
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.EE_PRE_GRASP_FAILED
                      
                      
    def apply_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying grasp with end effector")
        self.pub_move_robot_command.publish("grasp")   
        if wait_for_result("move_robot/e_out", 5, MoveRobotResult): 
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.EE_GRASP_FAILED
        
        
    def apply_release_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying release with end effector")
        self.pub_move_robot_command.publish("release")
        if wait_for_result("move_robot/e_out", 5, MoveRobotResult):     
            return PickPlaceResult.SUCCESS
        else:
            return PickPlaceResult.EE_RELEASE_FAILED
        
    def pick(self):
        rospy.logdebug("[PICK PLACE] Picking object")

        result = PickPlaceResult()
        attempts = 0
        while result != PickPlaceResult.SUCCESS:
            
            attempts += 1
            if attempts > 2:
                break
            
            result =  self.man_pre_grasp()
            if result != PickPlaceResult.SUCCESS:
                rospy.loginfo('trying transform pose with an additional 180deg')
                self.transform_pose(angle_offset = pi)

        if result == PickPlaceResult.SUCCESS:
            result = self.apply_pre_grasp_ee()
        
        if result == PickPlaceResult.SUCCESS:
            result = self.man_grasp()
        
        if result == PickPlaceResult.SUCCESS:
            result = self.apply_grasp_ee()

        if result == PickPlaceResult.SUCCESS:
            result = self.man_pre_grasp()         
         
        return result


    def place(self):
        rospy.logdebug("[PICK PLACE] Placing object")
        
        result = self.man_pre_place()

        if result == PickPlaceResult.SUCCESS:
            result = self.man_place()

        if result == PickPlaceResult.SUCCESS:
            result = self.apply_release_ee()

        if result == PickPlaceResult.SUCCESS:
            result = self.man_pre_place()

        if result == PickPlaceResult.SUCCESS:
            result = self.command_to_home()

        if result == PickPlaceResult.SUCCESS:
            result = self.reset_msg()
            
        return result
            
            
    def reset_msg(self):
        rospy.logdebug("[PICK PLACE] Resetting for next grasp")
        self.grasp_pose = None
        self.pre_grasp_pose = None
        self.pre_place_pose = None
        self.place_pose = None
        self.object_features = None
        return PickPlaceResult.SUCCESS            
            

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
        msg = PickPlaceResult()
        result = None

        # State dependent actions
        if self.state == "init":
            if self.command == "pick" or self.command == "pick_place":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be detected!")
                result = PickPlaceResult.FAILURE

        if self.command == "e_init":
            rospy.logdebug("[PICK PLACE] executing e_init command")
            result = PickPlaceResult.SUCCESS
            
        if self.command == "transform":
            rospy.logdebug("[PICK PLACE] executing transform command")
            result = self.transform_pose()
            
        if self.state == "idle":
            if self.command == "pick" or self.command == "pick_place":
                result = self.pick()
            elif self.command == "place":
                rospy.logwarn("Can not place object, it is not picked!")
                result = PickPlaceResult.FAILURE
        
        elif self.state == "picked":
            if self.command == "place" or self.command == "pick_place":
                result = self.place()
            elif self.command == "pick":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be placed!")
                result = PickPlaceResult.FAILURE
            
        elif self.command == "reset":
            result = self.reset_msg()

        success = result == PickPlaceResult.SUCCESS
        self.update_state(success)
        
        if self.command == "pick_place" and self.state == "picked" and success:
            success = None
            result = None

        

        # publish success
        if result is not None:

            msg.val = result
            pick_place_result_log(result)
            self.pub_e_out.publish(msg)            
            self.command = None
                      
                      
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
