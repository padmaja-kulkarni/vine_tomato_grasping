#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from flex_shared_resources.msg import SpawnInstruction


class StateMachineInput(object):

    def __init__(self, node_name):
        self.NODE_NAME = node_name
        self.command = None
        self.model_type = None

        self.sub_e_in = rospy.Subscriber("~e_in", SpawnInstruction, self.e_in_cb)
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

    def reset(self):
        """Resets the input state to its original state on init."""
        self.command = None
        self.model_type = None

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            if msg.type == SpawnInstruction.SPAWN:
                self.command = msg.type
                self.model_type = msg.model_type
                rospy.logdebug("[{0}] Received spawn command for truss type {1}".format(self.NODE_NAME, self.model_type))
            elif msg.type == SpawnInstruction.DELETE:
                self.command = msg.type
                rospy.logdebug("[{0}] Received delete command".format(self.NODE_NAME))
            else:
                rospy.logwarn("[{0}] Received unknown command {1}!".format(self.NODE_NAME, msg.type))
                self.command_rejected()

    def command_rejected(self):
        """Callback called when the state machine rejects the requested model."""
        rospy.logdebug("[%s] Rejected command in message", self.NODE_NAME)
        self.pub_e_out.publish(FlexGraspErrorCodes.FAILURE)
        self.reset()

    def command_accepted(self):
        """ method called when state machine accepts the requested command """
        rospy.logdebug("[%s] Accepted command in message: %s", self.NODE_NAME, self.command)
        msg = FlexGraspErrorCodes()
        msg.val = FlexGraspErrorCodes.NONE
        self.pub_e_out.publish(msg)

    def command_completed(self, success=True):
        """Callback called when the state machine rejects the requested model."""
        rospy.logdebug("[%s] Completed command in message: %s", self.NODE_NAME, self.command)
        if success:
            msg = FlexGraspErrorCodes.SUCCESS
        else:
            msg = FlexGraspErrorCodes.FAILURE
        self.pub_e_out.publish(msg)
        self.reset()
