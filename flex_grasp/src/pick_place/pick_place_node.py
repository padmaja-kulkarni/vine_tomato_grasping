#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

# from state_machine.state_machine_input import StateMachineInput
from state_machine.pick_place_state_machine import PickPlaceStateMachine
from pick_place import PickPlace


NODE_NAME = 'pick_place'
DEFAULT_UPDATE_RATE = 10.0
DEFAULT_DEBUG_MODE = True
DEFAULT_PLAYBACK = False


def main():
    debug_mode = rospy.get_param(NODE_NAME + "/debug", DEFAULT_DEBUG_MODE)
    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo("[{0}] Launching pick place node in debug mode".format(DEFAULT_DEBUG_MODE))
    else:
        log_level = rospy.INFO

    rospy.init_node(NODE_NAME, anonymous=True, log_level=log_level)
    update_rate = rospy.get_param('~update_rate', DEFAULT_UPDATE_RATE)
    playback = rospy.get_param("playback", DEFAULT_PLAYBACK)

    # state_machine_input = StateMachineInput(NODE_NAME)
    pick_place = PickPlace(NODE_NAME, playback=playback)
    pick_place_state_machine = PickPlaceStateMachine(pick_place, update_rate, NODE_NAME)
    rospy.loginfo('[{0}] Transform pose state machine successfully generated'.format(NODE_NAME))

    rospy.core.add_preshutdown_hook(lambda reason: pick_place_state_machine.request_shutdown())

    pick_place_state_machine.run()


if __name__ == '__main__':
    main()
