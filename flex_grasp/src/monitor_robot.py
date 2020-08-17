#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 17 16:03:22 2020

@author: taeke
"""

import rospy

from func.dynamixel_error import dynamixel_error_log

# services
from interbotix_sdk.srv import RegisterValues, RegisterValuesRequest
from interbotix_sdk.srv import RobotInfo, RobotInfoRequest

# msg
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
        
        
        self.rate = rospy.Rate(self.fequency)        

        self.get_robot_info = rospy.ServiceProxy('get_robot_info', RobotInfo)        
        self.get_motor_register_values = rospy.ServiceProxy('get_motor_register_values', RegisterValues)
    
        self.robot_info = self.get_robot_info()
        rospy.loginfo(self.robot_info)
        # self.joint_names =  robot_info.joint_names
        
        self.command = 1
        self.addres = "Shutdown"
        rospy.sleep(5)
        
    def get_register_values(self):
        request = RegisterValuesRequest()
        request.cmd = self.command
        request.addr_name = self.addres

        try:
            return self.get_motor_register_values(request)
        except:
            rospy.logwarn("Unable to get motor register values from request %s", request)
            return None

    def get_shutdown_error_codes(self, register_values):
        """" Gets error codes corresponding to the shutdown register values

        """
        if register_values is None:
            rospy.logwarn("[%s] Did not any shutdown register values!", self.node_name)
            return DynamixelErrorCodes.Failure
        
        # print self.robot_info.joint_names        
        
        if len(register_values.values) != self.robot_info.num_single_joints:
            rospy.logwarn("[%s] Did not receive shutdown error codes for all the joints!", self.node_name)
            return DynamixelErrorCodes.Failure

        shutdown_errors = {}    
    
        for register_value, joint_name in zip(register_values.values, self.robot_info.joint_names):
            shutdown_error = self.get_shutdown_error(register_value, joint_name)
            shutdown_errors[joint_name] = shutdown_error
            
            if shutdown_error.val is not DynamixelErrorCodes.SUCCESS:
                dynamixel_error_log(shutdown_error.val, joint_name, node_name=self.node_name)

        return shutdown_errors

    def get_shutdown_error(self, register_value, joint_name):
        """" Gets error codes corresponding to the shutdown register values

        """
        register_bin_string = '{0:08b}'.format(register_value)
        register_bin = [int(x) for x in list(register_bin_string)] # '{0:08b}'.format(register_value)
        if register_bin[7]:
            val = DynamixelErrorCodes.INPUT_VOLTAGE_ERROR
        elif not register_bin[5]:
            val = DynamixelErrorCodes.OVERHEATING_ERROR
        elif register_bin[4]:
            val = DynamixelErrorCodes.MOTOR_ENCODER_ERROR
        elif not register_bin[3]:
            val = DynamixelErrorCodes.ELECTRICAL_SHOCK_ERROR
        elif not register_bin[2]: # (default)
            val = DynamixelErrorCodes.OVERLOAD_ERROR
        else:
            val = DynamixelErrorCodes.SUCCESS

        result = DynamixelErrorCodes()
        # result.motor_name = joint_name  
        result.val = val  
        return result

    def monitor(self):
        register_values = self.get_register_values()
        
        error_codes = self.get_shutdown_error_codes(register_values)
            
def main():
    try:
        monitor_robot = MonitorRobot()

        while not rospy.core.is_shutdown():
            monitor_robot.monitor()
            monitor_robot.rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
