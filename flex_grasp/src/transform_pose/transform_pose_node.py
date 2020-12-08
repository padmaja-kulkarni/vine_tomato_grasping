#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from transform_pose import TransformPose


NODE_NAME = 'transform_pose'
DEFAULT_UPDATE_RATE = 10.0
DEFAULT_DEBUG_MODE = True


def main():
    debug_mode = rospy.get_param(NODE_NAME + "/debug", DEFAULT_DEBUG_MODE)
    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo("[%s] Launching transform pose node in debug mode", DEFAULT_DEBUG_MODE)
    else:
        log_level = rospy.INFO

    rospy.init_node(NODE_NAME, anonymous=True, log_level=log_level)

    update_rate = rospy.get_param('~update_rate', DEFAULT_UPDATE_RATE)
    transform_pose = TransformPose(NODE_NAME, update_rate)
    transform_pose.run()


if __name__ == '__main__':
    main()
