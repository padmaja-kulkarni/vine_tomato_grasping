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
    
    def __init__(self):
        self.state = "init"
        self.prev_state = None
        self.command = None
        
        
        self.debug_mode = rospy.get_param("pick_place/debug")
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[%s] Launching pick place node in debug mode", self.node_name)
        else:
            log_level = rospy.INFO
        
        rospy.init_node("pick_place", anonymous=True, log_level=log_level)
        self.rate = rospy.Rate(self.frequency)               
               
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)
        
        # init move robot communication
        move_robot_topic = "move_robot"
        self.move_robot_communication = Communication(move_robot_topic, timeout = 15)                   
                   
        self.pub_move_robot_pose = rospy.Publisher("robot_pose", PoseStamped, queue_size=10, latch=False)                   
                   
         # Subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        
        # create dict which stores all poses        
        keys = ['pre_grasp', 'grasp', 'pre_place','place']
        self.pose = dict.fromkeys(keys)
        
        # subscibe to corresponding poses
        values = [key + '_pose' for key in keys]
        pose_topic = dict(zip(keys, values))
        
        for key in pose_topic:
            rospy.Subscriber(pose_topic[key], PoseStamped, self.pose_in_cb, key)
        
        
        self.robot_base_frame = rospy.get_param('robot_base_frame')
        self.planning_frame = rospy.get_param('planning_frame')

    
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[PICK PLACE] Received command in message: %s", self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)    

    def pose_in_cb(self, msg, key):
        rospy.logdebug("[%s] Received %s pose", self.node_name, key)
        self.pose[key] = msg

    
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

    def fake_pick(self):
        rospy.logdebug("[PICK PLACE] Picking object")

        result = self.man_pre_grasp()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.apply_pre_grasp_ee()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_grasp()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.apply_grasp_ee()

        return result

    def fake_place(self):

        result = self.apply_release_ee()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.man_pre_grasp()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.command_to_home()

        if result == FlexGraspErrorCodes.SUCCESS:
            result = self.reset_msg()

        return result

    def pick(self):
        rospy.logdebug("[PICK PLACE] Picking object")
            
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
        rospy.loginfo("[PICK PLACE] updated pick place state, from %s to %s",
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

        elif (self.state == "idle") and ((self.command == "pick") or (self.command == "fake_pick") ) and success:
            self.prev_state = self.state
            self.state = "picked"
            self.log_state_update()

        elif (self.state == "picked") and (success == True or success == False):
            self.prev_state = self.state
            self.state = "init"
            self.log_state_update()                  
                  
                        
    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        # State dependent actions
        if self.state == "init" and not self.received_all_data():
            if self.command == "pick":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be detected!")
                result = FlexGraspErrorCodes.STATE_ERROR

        if self.command == "e_init":
            rospy.logdebug("[PICK PLACE] executing e_init command")
            result = FlexGraspErrorCodes.SUCCESS
            
        if self.state == "idle":
            if self.command == "pick":
                result = self.pick()
            if self.command == "fake_pick":
                result = self.fake_pick()
            elif self.command == "place":
                rospy.logwarn("Can not place object, it is not picked!")
                result = FlexGraspErrorCodes.STATE_ERROR
        
        elif self.state == "picked":
            if self.command == "place":
                result = self.place()
            elif self.command == "fake_place":
                result = self.fake_place()
            elif self.command == "pick":
                rospy.logwarn("[PICK PLACE] Can not pick object, it still needs to be placed!")
                result = FlexGraspErrorCodes.STATE_ERROR
            
        elif self.command == "reset":
            result = self.reset_msg()

        success = result == FlexGraspErrorCodes.SUCCESS
        if result == None:
            success = None
        self.update_state(success)

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
