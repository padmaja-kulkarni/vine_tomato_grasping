#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from flex_shared_resources.msg import GazeboInstruction


class GazeboInterfaceStateMachine(object):

    def __init__(self, gazebo_interface, input, update_rate, node_name):
        """Generates a state machine

        In order to start the state machine see `run`.

        :param StateMachineInput state_input: Input interface for controlling the states
        :param float update_rate: update rate in Hz
        """

        self.NODE_NAME = node_name
        self._update_rate = update_rate
        self._input = input
        self._gazebo_interface = gazebo_interface

        self._is_idle = True
        self._shutdown_requested = False
        self._command = None
        self._model_type = None

    def run(self):
        rate = rospy.Rate(self._update_rate)
        while not self._shutdown_requested:
            if self._is_idle:
                self._process_idle_state()
            else:
                self._process_spawn_state()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return

    def _process_idle_state(self):
        command = self._input.command

        if command is None:
            return

        if command == GazeboInstruction.SPAWN:
            model_type = self._input.model_type
            rospy.loginfo("[{0}] Requested to spawn {1} model".format(self.NODE_NAME, model_type))

            if model_type is None:
                rospy.logwarn('[{0}] Cannot spawn model: you forget to set its type!'.format(self.NODE_NAME))
                self._input.command_rejected()

            else:
                self._command = command
                self._model_type = model_type
                self._is_idle = False
                self._input.command_accepted()

        elif command == GazeboInstruction.DELETE:
            self._command = command
            self._is_idle = False
            self._input.command_accepted()

        elif command == GazeboInstruction.SETPOSE:
            self._command = command
            self._is_idle = False
            self._input.command_accepted()

        else:
            rospy.logwarn('[{0}] Cannot execute command {1}: command is unknown! Known commands are: {2}'.format(self.NODE_NAME, command, self._commands))
            self._input.command_rejected()

    def _process_spawn_state(self):

        if self._command == GazeboInstruction.SPAWN:
            rospy.logdebug("[%s] Executing spawn command", self.NODE_NAME)
            self._gazebo_interface.delete_all_models()
            success = self._gazebo_interface.spawn_model(self._model_type)
            self.command_completed(success)

        elif self._command == GazeboInstruction.DELETE:
            success = self._gazebo_interface.delete_all_models()
            self.command_completed(success)

        elif self._command == GazeboInstruction.SETPOSE:
            success = self._gazebo_interface.set_model_pose()
            self.command_completed(success)

        else:
            rospy.logwarn('`{0}` Unknown command `{1}`'.format(self.NODE_NAME, self._command))
            self.command_completed(False)

    def command_completed(self, success):
        rospy.loginfo("[{0}] Completed command {1}".format(self.NODE_NAME, self._command))
        self.reset()
        self._input.command_completed(success)


    def reset(self):
        """Resets the state machine state to its original state on init."""
        self._command = None
        self._model_type = None
        self._is_idle = True

    def request_shutdown(self):
        """Requests shutdown, which will terminate the state machine as soon as possible."""
        self._shutdown_requested = True
