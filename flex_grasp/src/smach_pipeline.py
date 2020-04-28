#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import String


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pick'])
        self.command_op_topic = "pipelineState"

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')

        self.command = rospy.wait_for_message(self.command_op_topic, String)

        if self.command.data == "PICK":
            return "pick"


class DetectObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'])
        self.detection_op_topic = "Object_Detection/e_out"

        self.pub_obj_detection = rospy.Publisher("Object_Detection/e_in",
                                      String, queue_size=10, latch=True)
        self.counter = 3
        self.timeout = 2.0
        self.object_detected = String()

    def execute(self, userdata):
        rospy.loginfo('Executing state Detect')

        # command node
        self.pub_obj_detection.publish("e_start")

        # get response
        try:
            self.object_detected = rospy.wait_for_message(self.detection_op_topic, String, self.timeout)
            rospy.logdebug("[PIPELINE] received object detected result: %s", self.object_detected.data)
        except:
            self.object_detected.data = 'failure'

        if self.object_detected.data == 'e_success':
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'


class PoseTransform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure','complete_failure'])
        self.tf_op_topic = "Pose_Transform/e_out"
        self.pub_pose_transform = rospy.Publisher("Pose_Transform/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 3.0
        self.counter = 3
        self.tf_result = String()

    def execute(self, userdata):
        rospy.loginfo('Executing state Transform')

        # command node
        self.pub_pose_transform.publish("e_start")

        # get response
        try:
            self.tf_result = rospy.wait_for_message(self.tf_op_topic, String, self.timeout)
            rospy.logdebug("[PIPELINE] received pose transform result: %s", self.tf_result.data)
        except:
            self.tf_result.data = 'failure'

        # determine success
        if self.tf_result.data == 'e_success':
            return 'success'
        else:
            self.counter = self.counter -1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'retry'])
        self.mv_robot_op_topic = "Move_Robot/e_out"
        self.pub_move_robot = rospy.Publisher("Move_Robot/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 3
        self.mv_robot_result = String()

    def execute(self, userdata):
        rospy.loginfo('Executing state Move Robot')

        # command node
        self.pub_move_robot.publish("pick")

        # get response
        try:
            self.mv_robot_result = rospy.wait_for_message(self.mv_robot_op_topic, String, self.timeout)
            rospy.logdebug("[PIPELINE] received move robot result: %s", self.mv_robot_result.data)
        except:
            self.mv_robot_result.data = 'failure'

        # determine success
        if self.mv_robot_result.data == 'e_success':
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'failure'
            return 'retry'



# main
def main():
    rospy.init_node('smach_state_machine',anonymous=True, log_level=rospy.DEBUG)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['total_success', 'total_failure'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'pick':'DetectObject'})
        smach.StateMachine.add('DetectObject', DetectObject(),
                               transitions={'success':'PoseTransform',
                                            'failure':'DetectObject',
                                            'complete_failure':'total_failure'})
        smach.StateMachine.add('PoseTransform', PoseTransform(),
                               transitions={'success':'MoveRobot',
                                            'failure':'DetectObject',
                                            'complete_failure':'total_failure'})
        smach.StateMachine.add('MoveRobot', MoveRobot(),
                               transitions={'success':'Idle',
                                            'failure':'Idle',
                                            'retry': 'MoveRobot'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
