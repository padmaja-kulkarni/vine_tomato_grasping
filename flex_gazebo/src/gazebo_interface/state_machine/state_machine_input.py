#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from flex_grasp.msg import FlexGraspErrorCodes
from flex_shared_resources.msg import GazeboInstruction


class StateMachineInput(object):

    def __init__(self, node_name):
        self.NODE_NAME = node_name
        self.command = None
        self.model_type = None

        self.sub_e_in = rospy.Subscriber("~e_in", GazeboInstruction, self.e_in_cb)
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

    def reset(self):
        """Resets the input state to its original state on init."""
        self.command = None
        self.model_type = None

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            if msg.command == GazeboInstruction.SPAWN:
                self.command = msg.command
                self.model_type = msg.model_type
                rospy.logdebug("[{0}] Received spawn command for truss type {1}".format(self.NODE_NAME, self.model_type))
            elif msg.command == GazeboInstruction.DELETE:
                self.command = msg.command
                rospy.logdebug("[{0}] Received delete command".format(self.NODE_NAME))
            elif msg.command == GazeboInstruction.SETPOSE:
                self.command = msg.command
                rospy.logdebug("[{0}] Received set pose command".format(self.NODE_NAME))
            else:
                rospy.logwarn("[{0}] Received unknown command {1}!".format(self.NODE_NAME, msg.command))
                self.command_rejected(FlexGraspErrorCodes.UNKNOWN_COMMAND)

    def command_rejected(self, error_code=FlexGraspErrorCodes.FAILURE):
        """method called when the state machine rejects the requested model."""
        rospy.logdebug("[%s] Rejected command in message", self.NODE_NAME)
        msg = FlexGraspErrorCodes(error_code)
        self.pub_e_out.publish(msg)
        self.reset()

    def command_accepted(self):
        """ method called when state machine accepts the requested command """
        rospy.logdebug("[%s] Accepted command in message: %s", self.NODE_NAME, self.command)
        msg = FlexGraspErrorCodes()
        msg.val = FlexGraspErrorCodes.NONE
        self.pub_e_out.publish(msg)

    def command_completed(self, success=True):
        """method called when the state machine rejects the requested model."""
        rospy.logdebug("[%s] Completed command in message: %s", self.NODE_NAME, self.command)
        if success:
            msg = FlexGraspErrorCodes.SUCCESS
        else:
            msg = FlexGraspErrorCodes.FAILURE
        self.pub_e_out.publish(msg)
        self.reset()
