#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 17 16:03:22 2020

@author: taeke
"""

import rospy
import numpy as np
from func.dynamixel_error import dynamixel_error_log
from func.flex_grasp_error import flex_grasp_error_log

# services
from interbotix_sdk.srv import RegisterValues, RegisterValuesRequest
from interbotix_sdk.srv import RobotInfo, RobotInfoRequest
from interbotix_sdk.srv import Reboot, RebootRequest
from std_srvs.srv import Empty

# msg
from std_msgs.msg import String
from flex_grasp.msg import FlexGraspErrorCodes
from flex_grasp.msg import DynamixelErrorCodes

class MonitorRobot(object):
    """MoveRobot"""
    node_name = "MONITOR ROBOT"    
    fequency = 10
    
    def __init__(self):
        
        self.debug_mode = rospy.get_param("monitor_robot/debug")        
        
        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[%s] Luanching move robot in debug mode", self.node_name)
        else:
            log_level = rospy.INFO

        rospy.init_node("move_robot",
                        anonymous=True,
                        log_level=log_level)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass
        
        self.command = None
        self.rate = rospy.Rate(self.fequency)        

        self.simulation  = self.debug_mode = rospy.get_param("robot_sim")
        
        if not self.simulation:
            rospy.wait_for_service("get_robot_info")
            self.get_robot_info = rospy.ServiceProxy('get_robot_info', RobotInfo)        
            self.get_motor_register_values = rospy.ServiceProxy('get_motor_register_values', RegisterValues)
            self.torque_joints_on = rospy.ServiceProxy('torque_joints_on', Empty)
            self.torque_joints_off = rospy.ServiceProxy('torque_joints_off', Empty)
            self.reboot_motor = rospy.ServiceProxy('reboot_motor', Reboot)
            
        
            robot_info = self.get_robot_info()
            rospy.loginfo(robot_info)
            self.all_joint_names = robot_info.joint_names
            self.gripper_joint_names = robot_info.joint_names[-1] 
            self.arm_joint_names = robot_info.joint_names[0:-1]
            
            self.addres = "Shutdown"
        
        rospy.Subscriber("~e_in", String, self.e_in_cb)        
        
        # Publishers
        self.pub_e_out = rospy.Publisher("~e_out",
                                   FlexGraspErrorCodes, queue_size=10, latch=True)        
        
    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[%s] Received new command: %s", self.node_name, self.command)

            # reset outputting message
            msg = FlexGraspErrorCodes()
            msg.val = FlexGraspErrorCodes.NONE
            self.pub_e_out.publish(msg)
        
    def reboot(self, joints = 'all'):
        """" Reboot Dynamixel servo(s)

        """
        if self.simulation:
            return FlexGraspErrorCodes.SUCCESS
            
        rospy.loginfo("[%s] Turning off all joint torque", self.node_name)
        self.torque_joints_off()
        
        rospy.loginfo("[%s] Rebooting Dynamixels", self.node_name)
        request = RebootRequest()
        request.cmd = 1
        self.reboot_motor(request)

        rospy.loginfo("[%s] Turning on all joint torque", self.node_name)        
        self.torque_joints_on()        
    
        return FlexGraspErrorCodes.SUCCESS
        
    def get_register_values(self, joints = 'all'):
        """" Read regirster value(s) from Dynamixel servo(s)

        """
        
        register_values = {}
        if joints == 'all':
            joint_names = self.all_joint_names
        elif joints == 'gripper':
            joint_names = self.gripper_joint_names
        elif joints == 'arm':
            joint_names = self.arm_joint_names
        else:
            joint_names = joints
    
        if not isinstance(joint_names, list): 
            joint_names = [joint_names]
    
        for joint_name in joint_names:
            request = RegisterValuesRequest()
            request.cmd = 4 # 4: single motor
            request.motor_name = joint_name
            request.addr_name = self.addres
    
            try:
                value = self.get_motor_register_values(request)
            except:
                rospy.logwarn("[%s] Unable to get motor register values from request %s", self.node_name, request)
                value = None
                
            register_values[joint_name] = value
        return register_values

    def get_dynamixel_error_codes(self, register_values):
        """" Gets error codes corresponding to the shutdown register values

        """

        dynamixel_errors = {}    
    
        for joint_name in register_values: # zip(register_values, self.robot_info.joint_names):
            register_value = register_values[joint_name]
            if register_value is None:
                dynamixel_error = DynamixelErrorCodes.COMMUNICATION_ERROR
            else:
                dynamixel_error = self.get_dynamixel_error(register_value.values[0], joint_name)
                
            dynamixel_errors[joint_name] = dynamixel_error
            if dynamixel_error is not DynamixelErrorCodes.SUCCESS:
                dynamixel_error_log(dynamixel_error, joint_name, node_name=self.node_name)

        return dynamixel_errors

    def get_dynamixel_error(self, register_value, joint_name):
        """" Gets error codes corresponding to the shutdown register values

        """
        register_bin_string = '{0:08b}'.format(register_value)
        register_bin = [int(x) for x in list(register_bin_string)] # '{0:08b}'.format(register_value)
        if register_bin[7]:
            val = DynamixelErrorCodes.INPUT_VOLTAGE_ERROR
        elif not register_bin[5]:
            val = DynamixelErrorCodes.OVERHEATING_ERROR
        elif not register_bin[3]:
            val = DynamixelErrorCodes.ELECTRICAL_SHOCK_ERROR
        elif register_bin[4]:
            val = DynamixelErrorCodes.MOTOR_ENCODER_ERROR
        elif not register_bin[2]: # (default)
            val = DynamixelErrorCodes.OVERLOAD_ERROR
        else:
            val = DynamixelErrorCodes.SUCCESS

        # result = DynamixelErrorCodes()
        # result.motor_name = joint_name  
        # result.val = val  
        return val # result

    def dynamixel_error_to_flex_grasp_error(self, dynamixel_errors):
        
        error_values = dynamixel_errors.values()       
        
        if np.all(DynamixelErrorCodes.SUCCESS == np.array(error_values)):
            return FlexGraspErrorCodes.SUCCESS    
            
        elif DynamixelErrorCodes.INPUT_VOLTAGE_ERROR in error_values:
            return FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR
            
        elif DynamixelErrorCodes.OVERHEATING_ERROR in error_values:
            return FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR   
            
        elif DynamixelErrorCodes.ELECTRICAL_SHOCK_ERROR in error_values:
            return FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR   
            
        elif DynamixelErrorCodes.MOTOR_ENCODER_ERROR in error_values:
            return FlexGraspErrorCodes.DYNAMIXEL_ERROR          
            
        elif DynamixelErrorCodes.OVERLOAD_ERROR in error_values:
            return FlexGraspErrorCodes.DYNAMIXEL_ERROR
            
        elif DynamixelErrorCodes.COMMUNICATION_ERROR in error_values: 
            return FlexGraspErrorCodes.DYNAMIXEL_ERROR
        else:
            return FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR
        
    def monitor(self, joints = 'gripper'):
        if self.simulation:
            return FlexGraspErrorCodes.SUCCESS
        
        register_values = self.get_register_values(joints = joints)
        dynamixel_errors = self.get_dynamixel_error_codes(register_values)
        result = self.dynamixel_error_to_flex_grasp_error(dynamixel_errors)  
        return result
        
    def take_action(self):
        msg = FlexGraspErrorCodes()
        result = None

        # actions
        if self.command == 'monitor_robot':
            result = self.monitor(joints = 'all')

        elif self.command == 'monitor_gripper':
            result = self.monitor(joints = 'gripper')

        elif self.command == 'monitor_arm':
            result = self.monitor(joints = 'arm')
            
        elif self.command == 'reboot':
           result = self.reboot()

        elif self.command == 'e_init':
            result = FlexGraspErrorCodes.SUCCESS
            
        elif self.command == None:
            pass
            
        else:
            rospy.logwarn("[%s] Received unknown command: %s", self.node_name, self.command)
            result = FlexGraspErrorCodes.UNKNOWN_COMMAND

        # publish result
        if result is not None:
            flex_grasp_error_log(result, node_name = self.node_name)
            msg.val = result
            self.pub_e_out.publish(msg)
            self.command = None        
        
def main():
    try:
        monitor_robot = MonitorRobot()

        while not rospy.core.is_shutdown():
            monitor_robot.take_action()
            monitor_robot.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
