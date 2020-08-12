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

from func.ros_utils import wait_for_variable, get_transform
from func.utils import add_pose_stamped
from func.conversions import pose_to_lists, point_to_pose_stamped, list_to_orientation, list_to_position
from func.flex_grasp_error import flex_grasp_error_log

from flex_grasp.msg import FlexGraspErrorCodes
from communication import Communication


import tf2_ros
import tf2_geometry_msgs

from math import pi

class PickPlace(object):
    
    node_name = 'PICK PLACE'    
    
    def __init__(self):
        self.state = "init"
        self.prev_state = None
        self.command = None
        
        self.peduncle_height = 0.075 # [m] 0.01 # 
        self.object_features = None
        
        
        self.debug_mode = rospy.get_param("pick_place/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[PICK PLACE] Launching pick place node in debug mode")
        else:
            log_level = rospy.INFO
        
        rospy.init_node("pick_place", anonymous=True, log_level=log_level)
        self.rate = rospy.Rate(10)               
               
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)
        
        self.pub_move_robot_pose = rospy.Publisher("robot_pose", PoseStamped, queue_size=10, latch=False)
        
        # init move robot communication
        move_robot_topic = "move_robot"
        self.move_robot_communication = Communication(move_robot_topic, timeout = 15)                   
                   
         # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("object_features", Truss, self.object_features_cb)

        self.use_iiwa = rospy.get_param('use_iiwa')
        self.use_interbotix = rospy.get_param('use_interbotix')
        
        self.robot_base_frame = rospy.get_param('robot_base_frame')
        self.planning_frame = rospy.get_param('planning_frame')


        if self.use_iiwa:
            rospy.logwarn("No pose transform for iiwa available!")        
        
        # Initialize Publishers
        pose_topic = {}
        pose_topic['pre_grasp'] = 'pre_grasp_pose'
        pose_topic['grasp'] = 'grasp_pose'        
        pose_topic['pre_place'] = 'pre_place_pose'        
        pose_topic['place'] = 'place_pose'
        
        pose_pub = {}
        for key in pose_topic:
            pose_pub[key] = rospy.Publisher(pose_topic[key], PoseStamped, queue_size=5, latch=True)
    

        grasp_xyz = [0, 0, 0.055] # [m]
        pre_grasp_xyz = [0, 0, 0.12] # [m]
        grasp_rpy = [-pi, pi/2, 0]
        place_rpy = [-pi, pi/2, 0.5]
        frame = self.robot_base_frame
        time = rospy.Time.now()

        pose_transform = {}
        pose_transform['pre_grasp'] = point_to_pose_stamped(pre_grasp_xyz, grasp_rpy, frame, time)
        pose_transform['grasp'] = point_to_pose_stamped(grasp_xyz, grasp_rpy, frame, time)
        pose_transform['pre_place'] = point_to_pose_stamped(pre_grasp_xyz, place_rpy, frame, time)
        pose_transform['place'] = point_to_pose_stamped(grasp_xyz, place_rpy, frame, time)

        pose = {}
        for key in pose_pub:
            pose[key] = None

        self.pose_pub = pose_pub
        self.pose = pose
        self.pose_transform = pose_transform

        # Tranform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
    
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[PICK PLACE] Received command in message: %s", self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)    
    
    def object_features_cb(self, msg):

        self.object_features = msg
        rospy.logdebug("[PICK PLACE] Received new object feature message")    

    def transform_pose(self, angle_offset = 0):
        if not wait_for_variable(3, self.object_features):
            rospy.logwarn("[PICK PLACE] Cannot transform pose, since object_features still empty!")
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED
            
        original_frame = self.object_features.cage_location.header.frame_id
        transform = get_transform(self.robot_base_frame, original_frame, self.tfBuffer)            
            
        if transform is None:
            rospy.logwarn("[PICK PLACE] Cannot transform pose, failed to lookup transform!!")
            return FlexGraspErrorCodes.TRANSFORM_POSE_FAILED

        object_pose = tf2_geometry_msgs.do_transform_pose(self.object_features.cage_location, transform)
        object_position, object_orientation = pose_to_lists(object_pose.pose, 'euler')
        object_pose.pose.position = list_to_position((object_position[0], object_position[1], self.peduncle_height))
        object_pose.pose.orientation = list_to_orientation((0, 0, object_orientation[2] + angle_offset))
        
        pose = {}
        for key in self.pose_transform:
            added_pose = add_pose_stamped(self.pose_transform[key], object_pose) 
            if added_pose is None:
                rospy.logwarn('[PICK PLACE] Failed to add pose stamed: they are defiend with respect to different frames!')
                return FlexGraspErrorCodes.Failure
            else:
                pose[key] = added_pose
            
            
        # tranform to world frame for MoveIt!
        transform = get_transform(self.planning_frame, self.robot_base_frame, self.tfBuffer) 
        for key in self.pose_transform:
            pose[key] = tf2_geometry_msgs.do_transform_pose(pose[key], transform)
            
        self.pose = pose

        
        self.pub_all_poses()
        return FlexGraspErrorCodes.SUCCESS

    def pub_all_poses(self):
        for key in self.pose_pub:
            self.pose_pub[key].publish(self.pose[key])
    
    def command_to_pose(self, pose):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pose")
        if pose is None:
            rospy.logwarn("[PICK PLACE] Cannot command to pose, since the pose is None!")
            return FlexGraspErrorCodes.REQUIRED_DATA_MISSING
        
        self.pub_move_robot_pose.publish(pose)
        return self.move_robot_communication.wait_for_result("move_manipulator")
        
    def man_pre_grasp(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pre grasp")
        return self.command_to_pose(self.pose['pre_grasp'])
            
    def man_grasp(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to grasp")
        return self.command_to_pose(self.pose['grasp'])
            
    def man_pre_place(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to pre place")
        return self.command_to_pose(self.pose['pre_place'])
                
    def man_place(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to place")
        return self.command_to_pose(self.pose['place'])
        
    def command_to_home(self):
        rospy.logdebug("[PICK PLACE] Commanding move robot to home")
        result = self.move_robot_communication.wait_for_result("home")
        return result

    def apply_pre_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying pre-grasp with end effector")
        result = self.move_robot_communication.wait_for_result("open")
        return result
                      
    def apply_grasp_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying grasp with end effector")
        result = self.move_robot_communication.wait_for_result("grasp")
        return result
        
    def apply_release_ee(self):
        rospy.logdebug("[PICK PLACE] Aplying release with end effector")
        result = self.move_robot_communication.wait_for_result("release")
        return result
        
    def pick(self):
        rospy.logdebug("[PICK PLACE] Picking object")
            
        result = self.man_pre_grasp()
        if result == FlexGraspErrorCodes.PLANNING_FAILED:
            rospy.loginfo('trying transform pose with an additional 180deg')
            self.transform_pose(angle_offset = pi)
            result = self.man_pre_grasp()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.apply_pre_grasp_ee()
        
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_grasp()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.apply_grasp_ee()
        
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_pre_grasp()     
         
        return result


    def place(self):
        rospy.logdebug("[PICK PLACE] Placing object")
        
        result = self.man_pre_place()
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_place()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.apply_release_ee()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_pre_place()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.command_to_home()
            
        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.reset_msg()
            
        return result
            
            
    def reset_msg(self):
        rospy.logdebug("[PICK PLACE] Resetting for next grasp")
        for key in self.pose:
            self.pose[key] = None
            
        self.object_features = None
        return FlexGraspErrorCodes.SUCCESS            
            

    def received_all_data(self):
        success = True
        for key in self.pose:
            if self.pose[key] is None:
                success = False
 
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
        msg = FlexGraspErrorCodes()
        result = None

        # State dependent actions
        if self.state == "init":
            if self.command == "pick" or self.command == "pick_place":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be detected!")
                result = FlexGraspErrorCodes.STATE_ERROR

        if self.command == "e_init":
            rospy.logdebug("[PICK PLACE] executing e_init command")
            result = FlexGraspErrorCodes.SUCCESS
            
        if self.command == "transform":
            rospy.logdebug("[PICK PLACE] executing transform command")
            result = self.transform_pose()
            
        if self.state == "idle":
            if self.command == "pick" or self.command == "pick_place":
                result = self.pick()
            elif self.command == "place":
                rospy.logwarn("Can not place object, it is not picked!")
                result = FlexGraspErrorCodes.STATE_ERROR
        
        elif self.state == "picked":
            if self.command == "place" or self.command == "pick_place":
                result = self.place()
            elif self.command == "pick":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be placed!")
                result = FlexGraspErrorCodes.STATE_ERROR
            
        elif self.command == "reset":
            result = self.reset_msg()

        success = result == FlexGraspErrorCodes.SUCCESS
        self.update_state(success)
        
        if self.command == "pick_place" and self.state == "picked" and success:
            result = None

        # publish success
        if result is not None:
            msg.val = result
            flex_grasp_error_log(result, self.node_name)
            self.pub_e_out.publish(msg)            
            self.command = None
                      
                      
def main():
    try:
        pick_place = PickPlace()
        while not rospy.core.is_shutdown():
            pick_place.take_action()
            pick_place.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
