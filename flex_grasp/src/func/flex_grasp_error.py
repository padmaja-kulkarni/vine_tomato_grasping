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
    else:
        return 'UNKNOWN_ERROR_CODE: still need to be added!'
        
def flex_grasp_error_log(val, node_name = None, mode = None):
    if val == FlexGraspErrorCodes.SUCCESS:
        if node_name is None:
            rospy.logdebug("Excuted succesfully")
        else:
            rospy.logdebug("[%s] Excuted succesfully", node_name)
    else:
        string = flex_grasp_error_string(val)
        
        if mode == 'debug':   
            if node_name is None:
                rospy.logdebug("Excuted with error: %s", string)
            else:
                rospy.logdebug("[%s] Excuted with error: %s", node_name, string)
        else:
            if node_name is None:
                rospy.logwarn("Excuted with error: %s", string)
            else:
                rospy.logwarn("[%s] Excuted with error: %s", node_name, string)