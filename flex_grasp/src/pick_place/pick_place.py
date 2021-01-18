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
from flex_grasp.msg import FlexGraspErrorCodes

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log
from flex_shared_resources.utils.communication import Communication



class PickPlace(object):
    
    node_name = 'PICK PLACE' 
    frequency = 10
    
    def __init__(self, node_name="pick_place", playback=False):

        self.node_name = node_name

        self.playback = playback
        if self.playback:
            rospy.loginfo("[{0}] Transform pose launched in playback mode!".format(self.node_name))

        self.state = "init"
        self.prev_state = None
        self.command = None

        # state machine input
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)
        
        # init move robot communication
        self.move_robot_communication = Communication("move_robot", timeout=15)
        self.pub_move_robot_pose = rospy.Publisher("robot_pose", PoseStamped, queue_size=10, latch=False)                   

        # create dict which stores all poses        
        keys = ['pre_grasp', 'grasp', 'pre_place', 'place']
        self.pose = dict.fromkeys(keys)
        
        # subscribe to corresponding poses
        values = [key + '_pose' for key in keys]
        pose_topic = dict(zip(keys, values))
        
        for key in pose_topic:
            rospy.Subscriber(pose_topic[key], PoseStamped, self.pose_in_cb, key)

    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[{0}] Received command in message: {1}".format(self.node_name, self.command))

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)    

    def pose_in_cb(self, msg, key):
        rospy.logdebug("[%s] Received %s pose", self.node_name, key)
        self.pose[key] = msg
    
    def command_to_pose(self, pose):
        rospy.logdebug("[{0}] Commanding move robot to pose".format(self.node_name))
        if pose is None:
            rospy.logwarn("[{0}] Cannot command to pose, since the pose is None!".format(self.node_name))
            return FlexGraspErrorCodes.REQUIRED_DATA_MISSING
        
        self.pub_move_robot_pose.publish(pose)
        return self.move_robot_communication.wait_for_result("move_manipulator")
        
    def man_pre_grasp(self):
        rospy.logdebug("[{0}] Commanding move robot to pre grasp".format(self.node_name))
        return self.command_to_pose(self.pose['pre_grasp'])
            
    def man_grasp(self):
        rospy.logdebug("[{0}] Commanding move robot to grasp".format(self.node_name))
        return self.command_to_pose(self.pose['grasp'])
            
    def man_pre_place(self):
        rospy.logdebug("[{0}] Commanding move robot to pre place".format(self.node_name))
        return self.command_to_pose(self.pose['pre_place'])
        
    def man_place(self):
        rospy.logdebug("[{0}] Commanding move robot to place".format(self.node_name))
        return self.command_to_pose(self.pose['place'])
        
    def command_to_home(self):
        rospy.logdebug("[{0}] Commanding move robot to home".format(self.node_name))
        result = self.move_robot_communication.wait_for_result("home")
        return result

    def apply_pre_grasp_ee(self):
        rospy.logdebug("[{0}] Applying pre-grasp with end effector".format(self.node_name))
        result = self.move_robot_communication.wait_for_result("open")
        return result
                      
    def apply_grasp_ee(self):
        rospy.logdebug("[{0}] Applying grasp with end effector".format(self.node_name))
        result = self.move_robot_communication.wait_for_result("grasp")
        return result
        
    def apply_release_ee(self):
        rospy.logdebug("[{0}] Applying release with end effector".format(self.node_name))
        result = self.move_robot_communication.wait_for_result("release")
        return result

    def pick(self):
        rospy.logdebug("[{0}] Picking object".format(self.node_name))
            
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
        rospy.logdebug("[{0}] Placing object".format(self.node_name))
        
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
        rospy.logdebug("[{0}] Resetting for next grasp".format(self.node_name))
        for key in self.pose:
            self.pose[key] = None

        return FlexGraspErrorCodes.SUCCESS            

    def received_all_data(self):
        success = True
        for key in self.pose:
            if self.pose[key] is None:
                success = False
        return success

    def log_state_update(self):
        rospy.loginfo("[{0}] updated pick place state, from {1} to {2}".format(self.node_name, self.prev_state, self.state))

    def update_state(self, success):

        if (self.state == "idle") and not self.received_all_data():
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()

        elif self.state == "init" and self.received_all_data():
            self.prev_state = self.state
            self.state = "idle"
            self.log_state_update()           

        elif (self.state == "idle") and (self.command == "pick") and success:
            self.prev_state = self.state
            self.state = "picked"
            self.log_state_update()

        elif (self.state == "picked") and (success is True or success is False):
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()                  

    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        # State dependent actions
        if self.state == "init" and not self.received_all_data():
            if self.command == "pick":
                rospy.logwarn("[{0}] Can not pick object, it still needs to be detected!".format(self.node_name))
                result = FlexGraspErrorCodes.STATE_ERROR

        if self.command == "e_init":
            rospy.logdebug("[{0}] executing e_init command".format(self.node_name))
            result = FlexGraspErrorCodes.SUCCESS
            
        if self.state == "idle":
            if self.command == "pick":
                result = self.pick()
            elif self.command == "place":
                rospy.logwarn("[{0}] Can not place object, it is not picked!".format(self.node_name))
                result = FlexGraspErrorCodes.STATE_ERROR
        
        elif self.state == "picked":
            if self.command == "place":
                result = self.place()
            elif self.command == "pick":
                rospy.logwarn("[{0}] Can not pick object, it still needs to be placed!".format(self.node_name))
                result = FlexGraspErrorCodes.STATE_ERROR
            
        elif self.command == "reset":
            result = self.reset_msg()

        success = (result == FlexGraspErrorCodes.SUCCESS)
        if result is None:
            success = None
        self.update_state(success)

        # publish success
        if result is not None:
            msg.val = result
            flex_grasp_error_log(result, self.node_name)
            self.pub_e_out.publish(msg)            
            self.command = None
