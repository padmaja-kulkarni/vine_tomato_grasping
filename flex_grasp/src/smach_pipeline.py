#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String

class Initializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'])
        self.timeout = 10.0

        self.pub_object_detection = rospy.Publisher("object_detection/e_in",
                                      String, queue_size=10, latch=True)
        self.pub_pose_transform = rospy.Publisher("pose_transform/e_in",
                                    String, queue_size=10, latch=True)
        self.pub_move_robot = rospy.Publisher("move_robot/e_in",
                                      String, queue_size=10, latch=True)
        self.pub_calibrate = rospy.Publisher("calibration_eye_on_base/calibrate/e_in",
                              String, queue_size=10, latch=True)

    def execute(self, userdata):
        rospy.logdebug('Executing state Initializing')

        init_object_detection = self.is_initialized("object_detection/e_out", self.pub_object_detection)
        init_pose_transform = self.is_initialized("pose_transform/e_out", self.pub_pose_transform)
        init_move_robot = self.is_initialized("move_robot/e_out", self.pub_move_robot)
        init_calibrate = self.is_initialized("calibration_eye_on_base/calibrate/e_out", self.pub_calibrate)

        if init_object_detection & init_pose_transform & init_move_robot & init_calibrate:
            return "success"
        else:
            rospy.logwarn("Failed to initialize")
            rospy.loginfo("init_object_detection %s", init_object_detection)
            rospy.loginfo("init_pose_transform %s", init_pose_transform)
            rospy.loginfo("init_move_robot %s", init_move_robot)
            rospy.loginfo("init_calibrate %s", init_calibrate)
            return "failure"

    def is_initialized(self, topic_out, topic_in):
        request = String()
        request.data = "e_init"
        topic_in.publish(request)
        return wait_for_success(topic_out, self.timeout)

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrate', 'detect', 'move', 'failure'], output_keys=['command'])
        self.command_op_topic = "pipelineState"

        self.detect_commands =  ["detect"]
        self.calibrate_commands =  ["calibrate"]
        self.move_commands =  ["home", "open", "close", "pick", "place", "pick_place"]

    def execute(self, userdata):
        rospy.logdebug('Executing state Idle')

        command = rospy.wait_for_message(self.command_op_topic, String).data

        if command in self.detect_commands:
            return "detect"
        elif command in self.move_commands:
            userdata.command = command
            return "move"
        elif command in self.calibrate_commands:
            userdata.command = command
            return "calibrate"
        else:
            rospy.logwarn('Unknown command: %s', command)
            return "failure"


class DetectObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'])
        self.detection_op_topic = "object_detection/e_out"

        self.pub_obj_detection = rospy.Publisher("object_detection/e_in",
                                      String, queue_size=10, latch=True)
        self.counter = 3
        self.timeout = 5.0
        self.object_detected = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Detect')

        # command node
        self.pub_obj_detection.publish("e_start")

        # get response
        success = wait_for_success(self.detection_op_topic, self.timeout)

        if success:
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class CalibrateRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'])
        self.calibrate_topic = "calibration_eye_on_base/calibrate/e_out"

        self.pub_calibrate = rospy.Publisher("calibration_eye_on_base/calibrate/e_in",
                                      String, queue_size=10, latch=True)
        self.counter = 3
        self.timeout = 30.0

    def execute(self, userdata):
        rospy.logdebug('Executing state Calibrate')

        # command node
        self.pub_calibrate.publish("e_start")

        # get response
        success = wait_for_success(self.calibrate_topic, self.timeout)

        if success:
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class PoseTransform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure','complete_failure'])
        self.tf_op_topic = "pose_transform/e_out"
        self.pub_pose_transform = rospy.Publisher("pose_transform/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 40.0
        self.counter = 3
        self.tf_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Transform')

        # command node
        self.pub_pose_transform.publish("e_start")

        # get response
        success = wait_for_success(self.tf_op_topic, self.timeout)

        # determine success
        if success:
            return 'success'
        else:
            self.counter = self.counter -1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'retry'], input_keys=['command'])
        self.mv_robot_op_topic = "move_robot/e_out"
        self.pub_move_robot = rospy.Publisher("move_robot/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 3
        self.mv_robot_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Move Robot')

        # command node
        self.pub_move_robot.publish(userdata.command)

        # get response
        success = wait_for_success(self.mv_robot_op_topic, self.timeout)

        # determine success
        if success:
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'failure'
            return 'retry'

def wait_for_success(topic, timeout):


    start_time = rospy.get_time()
    curr_time = rospy.get_time()

    # rospy.logdebug("==WAITING FOR SUCCESS==")
    # rospy.logdebug("start time: %s", start_time)
    # rospy.logdebug("current time: %s", curr_time)
    while (curr_time - start_time < timeout): # and not rospy.is_shutdown():
        # rospy.logdebug("current time: %s", curr_time)
        try:
            message = rospy.wait_for_message(topic, String, timeout)
            if message.data == "e_success":
                rospy.logdebug("[PIPELINE] Command succeeded: received %s on topic %s", message.data, topic)
                return True
            elif message.data == "":
                pass
            else:
                rospy.logwarn("[PIPELINE] Command failed: node returned %s on topic %s", message.data, topic)
                return False
        except:
            rospy.logwarn("[PIPELINE] Command failed: timeout exceeded while waiting for message on topic %s", topic)
            return False

        rospy.sleep(0.2)
        curr_time = rospy.get_time()

    rospy.logwarn("[PIPELINE] Command failed: node did not return success within timeout on topic %s", topic)
    return False

# main
def main():
    debug_mode = rospy.get_param("pipeline/debug")

    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo("[PIPELINE] Luanching pipeline node in debug mode")
    else:
        log_level = rospy.INFO

    rospy.init_node('smach_state_machine',anonymous=True, log_level=log_level)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['total_success', 'total_failure'])

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('pipeline_server', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initializing', Initializing(),
                               transitions={'success':'Idle',
                                            'failure':'Initializing',
                                            'complete_failure':'total_failure'})
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'detect':'DetectObject',
                                            'move': 'MoveRobot',
                                            'calibrate': 'CalibrateRobot',
                                            'failure': 'Idle'})

        smach.StateMachine.add('CalibrateRobot', CalibrateRobot(),
                           transitions={'success':'Idle',
                                        'failure': 'Idle',
                                        'complete_failure':'Idle'})

        smach.StateMachine.add('DetectObject', DetectObject(),
                               transitions={'success':'PoseTransform',
                                            'failure':'DetectObject',
                                            'complete_failure':'Idle'})

        smach.StateMachine.add('PoseTransform', PoseTransform(),
                               transitions={'success':'Idle',
                                            'failure':'DetectObject',
                                            'complete_failure':'Idle'})

        smach.StateMachine.add('MoveRobot', MoveRobot(),
                               transitions={'success':'Idle',
                                            'failure':'Idle',
                                            'retry': 'MoveRobot'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
