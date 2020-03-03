#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  3 16:03:30 2020

@author: jelle
"""

import rospy
import smach

# define state Foo
class detectObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state detectObject')
        if self.counter < 3:
            self.counter += 1
            return 'succeeded'
        else:
            return 'failed'


# define state Bar
class moveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state moveRobot')
        if self.counter < 3:
            self.counter += 1
            return 'succeeded'
        else:
            return 'failed'



# main
def main():
    rospy.init_node('StateMachine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('detectObject', detectObject(), 
                               transitions={'succeeded':'moveRobot', 
                                            'failed':'failed'})
        smach.StateMachine.add('moveRobot', moveRobot(), 
                               transitions={'succeeded':'detectObject',
                                            'failed':'detectObject'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()