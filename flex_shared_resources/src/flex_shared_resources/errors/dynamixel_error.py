# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 15:52:00 2020

@author: taeke
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 09:21:25 2020

@author: taeke
"""
from flex_grasp.msg import DynamixelErrorCodes
import rospy

def dynamixel_error_string(val):
    """Returns a string associated with a DynamixelErrorCodes.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == DynamixelErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == DynamixelErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == DynamixelErrorCodes.NONE:
        return 'NONE'
        
    # general errors
    elif val == DynamixelErrorCodes.OVERLOAD_ERROR:
        return 'OVERLOAD_ERROR'
    elif val == DynamixelErrorCodes.ELECTRICAL_SHOCK_ERROR:
        return 'ELECTRICAL_SHOCK_ERROR'
    elif val == DynamixelErrorCodes.MOTOR_ENCODER_ERROR:
        return 'MOTOR_ENCODER_ERROR'
    elif val == DynamixelErrorCodes.OVERHEATING_ERROR:
        return 'OVERHEATING_ERROR'
    elif val == DynamixelErrorCodes.INPUT_VOLTAGE_ERROR:
        return 'INPUT_VOLTAGE_ERROR'
        
    elif val == DynamixelErrorCodes.COMMUNICATION_ERROR:
        return 'COMMUNICATION_ERROR'


    else:
        return 'UNKNOWN_ERROR_CODE: still need to be added!'
        
def dynamixel_error_log(val, motor_name, node_name = None, mode = None):
    if val == DynamixelErrorCodes.SUCCESS:
        if node_name is None:
            rospy.logdebug("Excuted succesfully")
        else:
            rospy.logdebug("[%s] %s did not return an error", motor_name, node_name)
    else:
        string = dynamixel_error_string(val)
        
        if mode == 'debug':   
            if node_name is None:
                rospy.logdebug("%s  returned error: %s", motor_name, string)
            else:
                rospy.logdebug("[%s] %s returned error: %s", node_name, motor_name, string)
        else:
            if node_name is None:
                rospy.logwarn("%s returned error: %s", motor_name, string)
            else:
                rospy.logwarn("[%s] %s returned error: %s", node_name,  motor_name, string)