# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 17:36:32 2020

@author: taeke
"""

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
from flex_grasp.msg import PickPlaceResult
import rospy

def pick_place_result_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == PickPlaceResult.SUCCESS:
        return 'SUCCESS'
    elif val == PickPlaceResult.FAILURE:
        return 'FAILURE'
    elif val == PickPlaceResult.NONE:
        return 'NONE'
        
    elif val == PickPlaceResult.MAN_PRE_GRASP_FAILED:
        return 'MAN_PRE_GRASP_FAILED'
    elif val == PickPlaceResult.MAN_GRASP_FAILED:
        return 'MAN_GRASP_FAILED'
    elif val == PickPlaceResult.MAN_PRE_PLACE_FAILED:
        return 'MAN_PRE_PLACE_FAILED'
    elif val == PickPlaceResult.MAN_PLACE_FAILED:
        return 'MAN_PLACE_FAILED'
        
    elif val == PickPlaceResult.MAN_HOME_FAILED:
        return 'MAN_HOME_FAILED'
        
    elif val == PickPlaceResult.EE_GRASP_FAILED:
        return 'EE_GRASP_FAILED'
    elif val == PickPlaceResult.EE_PRE_GRASP_FAILED:
        return 'EE_PRE_GRASP_FAILED'
    elif val == PickPlaceResult.EE_RELEASE_FAILED:
        return 'EE_RELEASE_FAILED'
        
    elif val == PickPlaceResult.TRANSFORM_POSE_FAILED:
        return 'TRANSFORM_POSE_FAILED'    
        
    else:
        return 'UNKNOWN_ERROR_CODE: add this error to pick_place_result_string()'
        
def pick_place_result_log(val):
    if val == PickPlaceResult.SUCCESS:
        rospy.logdebug("Move robot excuted succesfully")
    else:
        string = pick_place_result_string(val)
        rospy.logwarn("Move robot excuted with error: %s", string)