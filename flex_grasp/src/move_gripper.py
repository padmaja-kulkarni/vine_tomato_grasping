#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 12:06:31 2020

@author: taeke
"""

from interbotix_sdk.msg import SingleCommand
from interbotix_sdk.srv import OperatingModes, OperatingModesRequest
from interbotix_sdk.srv import RobotInfo
from sensor_msgs.msg import JointState

from flex_grasp.msg import FlexGraspErrorCodes
import rospy

import moveit_commander



class MoveGripper(object):
    """MoveGripper"""
    node_name = "MOVE_GRIPPER"   
    frequency = 20
    
    def __init__(self):
        
        self.debug_mode = True # rospy.get_param("move_robot/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[%s] Luanching move robot in debug mode", self.node_name)
        else:
            log_level = rospy.INFO

        rospy.init_node("move_gripper",
                        anonymous=True,
                        log_level=log_level)
        self.rate = rospy.Rate(self.frequency)     

        
        rospy.wait_for_service("get_robot_info")
        srv_robot_info = rospy.ServiceProxy("get_robot_info", RobotInfo)
        self.resp = srv_robot_info()
        self.num_joints = self.resp.num_joints
        self.gripper_index = self.num_joints + 1
        rospy.loginfo(self.resp)
        
        self.set_operating_mode_srv = rospy.ServiceProxy('set_operating_modes', OperatingModes)
        self.pub_gripper_command = rospy.Publisher("single_joint/command",
                                   SingleCommand, queue_size=10, latch=True) 
         
        self.sub_joint_states = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)

        self.gripper_command = SingleCommand()   
        self.gripper_command.joint_name = 'gripper'
        self.gripper_command.cmd = 0
        
        self.joint_states = JointState()
                          
        self.set_operating_mode()
        self.state = 'open'

        self.close_pwm_cmd = -200
        self.open_pwm_cmd = 200

        rospy.logdebug('[%s] Done initializing', self.node_name)
        
        ee_group = moveit_commander.MoveGroupCommander('interbotix_gripper')        
        
        self.EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        self.EE_OPEN = ee_group.get_named_target_values("Open").values()
        
    ### @brief ROS Subscriber Callback function to update the latest arm joint states
    ### @param msg - latest JointState message
    ### @details - the JointState message is mainly used to determine current gripper position
    def joint_state_cb(self, msg):
        self.joint_states = msg                                   
      
    def set_operating_mode(self):
        request = OperatingModesRequest()
        request.cmd = 3 # gripper
        request.mode = 'pwm'
        request.use_custom_profiles = False
        self.set_operating_mode_srv(request)     
                             
    def commander(self, command, timeout = 2.0):
        self.gripper_command.cmd = command
        self.pub_gripper_command.publish(self.gripper_command)
        
        
        start_time = rospy.get_time()
        curr_time = rospy.get_time()        
        
        while (curr_time - start_time < timeout) and not rospy.is_shutdown():
            js_msg = list(self.joint_states.position)
            if len(js_msg) != 0:
           
           
                if ((self.gripper_command.cmd > 0 and js_msg[self.gripper_index] >= self.EE_OPEN[0]) or
                   (self.gripper_command.cmd < 0 and js_msg[self.gripper_index] <= self.EE_CLOSED[0])):
                    self.gripper_command.cmd = 0
                    self.pub_gripper_command.publish(self.gripper_command)
                    return FlexGraspErrorCodes.SUCCESS   
                
            curr_time = rospy.get_time()   
            self.rate.sleep()

        if rospy.is_shutdown():
            return FlexGraspErrorCodes.SHUTDOWN

        rospy.logwarn("[%s], Control failed: gripper did not reach target within allocated time", self.node_name)
        return FlexGraspErrorCodes.CONTROL_FAILED        
                
    def close_pwm(self):
        rospy.logdebug("[%s] Closing end effector", self.node_name)
        self.set_operating_mode()
        self.commander(self.close_pwm_cmd)
                
    def open_pwm(self):
        rospy.logdebug("[%s] Opening end effector", self.node_name)
        self.set_operating_mode()
        self.commander(self.open_pwm_cmd)                
                
    def take_action(self):
        if self.state == 'open':
            self.close_pwm()
            self.state = 'close'
            
        elif self.state == 'close':
            self.open_pwm()
            self.state = 'open'            
                
def main():
    try:
        move_gripper = MoveGripper()
        rospy.sleep(1.0)
        while not rospy.core.is_shutdown():
            move_gripper.take_action()
            move_gripper.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()