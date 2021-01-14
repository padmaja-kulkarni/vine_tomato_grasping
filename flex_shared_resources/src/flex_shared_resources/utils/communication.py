# -*- coding: utf-8 -*-
"""
Created on Wed Aug 12 08:20:26 2020

@author: taeke
"""


import rospy
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log

class Communication(object):
    """ObjectDetection"""
    
    e_in = "e_in"
    e_out = "e_out"
    
    def __init__(self, topic, timeout=30, frequency=10, msg_type=String, node_name="undefined"):
        self.topic = topic
        self.timeout = timeout
        self.frequency = frequency
        self.node_name = node_name
        
        self.rate = rospy.Rate(self.frequency)
        self.result = FlexGraspErrorCodes.NONE
        self.pub_e_in = rospy.Publisher(topic + "/" + self.e_in, msg_type, queue_size=10, latch=True)
        rospy.Subscriber(topic + "/" + self.e_out, FlexGraspErrorCodes, self.e_out_cb)
                                
    def e_out_cb(self, msg):
        if self.result is None:
            if msg.val != FlexGraspErrorCodes.NONE:
                flex_grasp_error_log(msg.val, self.node_name, mode='debug')
                self.result = msg.val

    def wait_for_result(self, command):
        
        rospy.logdebug("%s, Publishing command %s on topic %s", self.node_name, command, self.topic)
        self.pub_e_in.publish(command)
        self.result = None
        
        start_time = rospy.get_time()
        curr_time = rospy.get_time()

        while (curr_time - start_time < self.timeout) and not rospy.is_shutdown():
            if self.result is not None:

                if self.result == FlexGraspErrorCodes.NONE:
                    pass
                if self.result == FlexGraspErrorCodes.SUCCESS:
                    rospy.logdebug("[{0}] Command succeeded: received success on topic {1}".format(self.node_name, self.topic))
                    return self.result
                else:
                    rospy.logdebug("[{0}] Command failed: node returned {1} on topic {2}".format(self.node_name, self.result, self.topic))
                    return self.result
            self.rate.sleep()
            curr_time = rospy.get_time()
    
        if rospy.is_shutdown():
            return FlexGraspErrorCodes.SHUTDOWN

        rospy.logwarn("[{0}] Command failed: node did not return a message within timeout on topic {1}".format(self.node_name, self.topic))
        return FlexGraspErrorCodes.TIMEOUT        
