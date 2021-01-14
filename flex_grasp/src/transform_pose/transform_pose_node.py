#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy

from state_machine.state_machine_input import StateMachineInput
from state_machine.transform_pose_state_machine import TransformPoseStateMachine
from transform_pose import TransformPose


NODE_NAME = 'transform_pose'
DEFAULT_UPDATE_RATE = 10.0
DEFAULT_DEBUG_MODE = True
DEFAULT_PLAYBACK = False

def main():
    debug_mode = rospy.get_param(NODE_NAME + "/debug", DEFAULT_DEBUG_MODE)
    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo("[%s] Launching transform pose node in debug mode", DEFAULT_DEBUG_MODE)
    else:
        log_level = rospy.INFO

    rospy.init_node(NODE_NAME, anonymous=True, log_level=log_level)
    update_rate = rospy.get_param('~update_rate', DEFAULT_UPDATE_RATE)
    playback = rospy.get_param("playback", DEFAULT_PLAYBACK)

    state_machine_input = StateMachineInput(NODE_NAME)
    transform_pose = TransformPose(NODE_NAME, playback=playback)
    transform_pose_state_machine = TransformPoseStateMachine(transform_pose, state_machine_input, update_rate, NODE_NAME)
    rospy.loginfo('[%s] Transform pose state machine successfully generated', NODE_NAME)

    rospy.core.add_preshutdown_hook(lambda reason: transform_pose_state_machine.request_shutdown())

    transform_pose_state_machine.run()


if __name__ == '__main__':
    main()
