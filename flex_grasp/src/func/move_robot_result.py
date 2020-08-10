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
from flex_grasp.msg import MoveRobotResult
import rospy

def move_robot_result_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveRobotResult.SUCCESS:
        return 'SUCCESS'
    elif val == MoveRobotResult.FAILURE:
        return 'FAILURE'
    elif val == MoveRobotResult.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveRobotResult.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveRobotResult.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    elif val == MoveRobotResult.INVALID_FRAME:
        return 'INVALID_FRAME'
    elif val == MoveRobotResult.NO_GOAL_POSE:
        return 'NO_GOAL_POSE'
    elif val == MoveRobotResult.UNKNOWN_COMMAND:
        return 'UNKNOWN_COMMAND'
    else:
        return 'UNKNOWN_ERROR_CODE'
        
def move_robot_result_log(val):
    if val == MoveRobotResult.SUCCESS:
        rospy.logdebug("Move robot excuted succesfully")
    else:
        string = move_robot_result_string(val)
        rospy.logwarn("Move robot excuted with error: %s", string)