#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 15 12:56:46 2020

@author: taeke
"""

import rospy

# messages
from std_msgs.msg import String

def wait_for_variable(timeout, variable):
    start_time = rospy.get_time()

    while (rospy.get_time() - start_time < timeout):   
        
        if variable is not None:  
            return True
            
        rospy.sleep(0.1)
    
    return False  


def wait_for_success(topic, timeout):


    start_time = rospy.get_time()
    curr_time = rospy.get_time()

    # rospy.logdebug("==WAITING FOR SUCCESS==")
    # rospy.logdebug("start time: %s", start_time)
    # rospy.logdebug("current time: %s", curr_time)
    while (curr_time - start_time < timeout) and not rospy.is_shutdown():
        # rospy.logdebug("current time: %s", curr_time)
        try:
            message = rospy.wait_for_message(topic, String, timeout)
            if message.data == "e_success":
                rospy.logdebug("Command succeeded: received %s on topic %s", message.data, topic)
                return True
            elif message.data == "":
                pass
            else:
                rospy.logwarn("Command failed: node returned %s on topic %s", message.data, topic)
                return False
        except:
            rospy.logwarn("Command failed: timeout exceeded while waiting for message on topic %s", topic)
            return False

        rospy.sleep(0.1)
        curr_time = rospy.get_time()

    if rospy.is_shutdown():
        pass
    else:
        rospy.logwarn("Command failed: node did not return success within timeout on topic %s", topic)
    return False
    
    
def wait_for_result(topic, timeout, msg_type):


    start_time = rospy.get_time()
    curr_time = rospy.get_time()

    # rospy.logdebug("==WAITING FOR SUCCESS==")
    # rospy.logdebug("start time: %s", start_time)
    # rospy.logdebug("current time: %s", curr_time)
    while (curr_time - start_time < timeout) and not rospy.is_shutdown():
        # rospy.logdebug("current time: %s", curr_time)
        try:
            message = rospy.wait_for_message(topic, msg_type, timeout)
            if message.val == msg_type.NONE:
                pass
            elif message.val == msg_type.SUCCESS:
                rospy.logdebug("Command succeeded: received success on topic %s", topic)
                return True
            else:
                rospy.logwarn("Command failed: node returned %s on topic %s", message.val, topic)
                return False
        except:
            rospy.logwarn("Command failed: timeout exceeded while waiting for message on topic %s", topic)
            # return msg_type.TIMEOUT      
            return False

        rospy.sleep(0.1)
        curr_time = rospy.get_time()

    if rospy.is_shutdown():
        pass
    else:
        rospy.logwarn("Command failed: node did not return success within timeout on topic %s", topic)
    return False
    
def wait_for_param(param, timeout):

    start_time = rospy.get_time()

    while (rospy.get_time() - start_time < timeout):

        if rospy.has_param(param):
            return rospy.get_param(param)

    rospy.logwarn("[MOVE ROBOT] Parameter %s can not be loaded from parameter server: timeout passed", param)
    return None


def get_transform(to_frame, from_frame, tfBuffer):
    try:
        
        if (from_frame is None) and (to_frame is None):      
            rospy.logwarn("Cannot find transform, both frames are not defined!")
            return None  
        
        if from_frame is None:
            rospy.logwarn("Cannot find transform to %s, no header is defined!", to_frame)
            return None        
        
        if to_frame is None:
            rospy.logwarn("Cannot find transform from %s, no frame is defined!", from_frame)
            return None
            

            
        trans = tfBuffer.lookup_transform(to_frame, from_frame, time = rospy.Time.now())
        return trans
        
    except:
        return None