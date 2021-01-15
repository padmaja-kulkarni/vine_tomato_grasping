#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

# from state_machine.state_machine_input import StateMachineInput
from state_machine.calibration_state_machine import CalibrationStateMachine
from calibration import Calibration


NODE_NAME = 'calibration'
DEFAULT_UPDATE_RATE = 10.0
DEFAULT_DEBUG_MODE = False
DEFAULT_PLAYBACK = False


def main():
    debug_mode = rospy.get_param(NODE_NAME + "/debug", DEFAULT_DEBUG_MODE)
    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo("[{0}] Launching calibration node in debug mode".format(NODE_NAME))
    else:
        log_level = rospy.INFO

    rospy.init_node(NODE_NAME, anonymous=True, log_level=log_level)
    update_rate = rospy.get_param('~update_rate', DEFAULT_UPDATE_RATE)
    playback = rospy.get_param('/' + 'px150' + '/' + 'playback', DEFAULT_PLAYBACK)

    # state_machine_input = StateMachineInput(NODE_NAME)
    calibration = Calibration(NODE_NAME, playback=playback)
    calibration_state_machine = CalibrationStateMachine(calibration, update_rate, NODE_NAME)
    rospy.loginfo('[{0}] Calibration state machine successfully generated'.format(NODE_NAME))

    rospy.core.add_preshutdown_hook(lambda reason: calibration_state_machine.request_shutdown())

    calibration_state_machine.run()


if __name__ == '__main__':
    main()
