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
from flex_grasp.msg import FlexGraspErrorCodes
import rospy

def flex_grasp_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == FlexGraspErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == FlexGraspErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == FlexGraspErrorCodes.NONE:
        return 'NONE'
        
    # general errors
    elif val == FlexGraspErrorCodes.UNKNOWN_COMMAND:
        return 'UNKNOWN_COMMAND'
    elif val == FlexGraspErrorCodes.TIMEOUT:
        return 'TIMEOUT'
    elif val == FlexGraspErrorCodes.STATE_ERROR:
        return 'STATE_ERROR'
    elif val == FlexGraspErrorCodes.SHUTDOWN:
        return 'SHUTDOWN'
    elif val == FlexGraspErrorCodes.REQUIRED_DATA_MISSING:
        return 'REQUIRED_DATA_MISSING'

    elif val == FlexGraspErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'        
    elif val == FlexGraspErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == FlexGraspErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    elif val == FlexGraspErrorCodes.INVALID_FRAME:
        return 'INVALID_FRAME'
    elif val == FlexGraspErrorCodes.NO_GOAL_POSE:
        return 'NO_GOAL_POSE'
        
    elif val == FlexGraspErrorCodes.TRANSFORM_POSE_FAILED:
        return 'TRANSFORM_POSE_FAILED'
    
    elif val == FlexGraspErrorCodes.DYNAMIXEL_ERROR:
        return 'DYNAMIXEL_ERROR'
    elif val == FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR:      
        return 'DYNAMIXEL_SEVERE_ERROR'
    else:
        return 'UNKNOWN_ERROR_CODE: still need to be added!'
        
def flex_grasp_error_log(val, node_name = None, mode = None):
    error_string = flex_grasp_error_string(val)

    if node_name is None:
        if val == FlexGraspErrorCodes.SUCCESS:
            log_string = "Excuted with {0}".format(error_string)
        else:
            log_string = "Excuted with error {0}".format(error_string)
    else:
        if val == FlexGraspErrorCodes.SUCCESS:
            log_string = "[{0}] Excuted with {1}".format(node_name, error_string)
        else:
            log_string = "[{0}] Excuted with error {1}".format(node_name, error_string)


    if val == FlexGraspErrorCodes.SUCCESS or mode == 'debug':
        rospy.logdebug(log_string)
    else:
        rospy.logwarn(log_string)
