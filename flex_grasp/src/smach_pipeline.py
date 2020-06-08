#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String
from func.ros_utils import wait_for_success


class Initializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'])
        self.timeout = 10.0

        self.pub_object_detection = rospy.Publisher("object_detection/e_in",
                                      String, queue_size=10, latch=True)
        self.pub_pose_transform = rospy.Publisher("pick_place/e_in",
                                    String, queue_size=10, latch=True)
        self.pub_move_robot = rospy.Publisher("move_robot/e_in",
                                      String, queue_size=10, latch=True)
        self.pub_calibrate = rospy.Publisher("calibration_eye_on_base/calibrate/e_in",
                              String, queue_size=10, latch=True)

    def execute(self, userdata):
        rospy.logdebug('Executing state Initializing')

        init_object_detection = self.is_initialized("object_detection/e_out", self.pub_object_detection)
        init_pose_transform = self.is_initialized("pick_place/e_out", self.pub_pose_transform)
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
        smach.State.__init__(self, outcomes=['calibrate', 'detect', 'move', "pick_place", "point", 'failure'], output_keys=['command'])
        self.command_op_topic = "pipelineState"

        self.detect_commands =  ["detect_tomato", "detect_truss"]
        self.calibrate_commands =  ["calibrate"]
        self.move_commands =  ["home", "open", "close"]
        self.pick_place_commands = ["pick", "place", "pick_place"]
        self.point_commands = ["point"]

    def execute(self, userdata):
        rospy.logdebug('Executing state Idle')

        command = rospy.wait_for_message(self.command_op_topic, String).data

        if command in self.detect_commands:
            userdata.command = command            
            return "detect"
        elif command in self.move_commands:
            userdata.command = command
            return "move"
        elif command in self.calibrate_commands:
            userdata.command = command
            return "calibrate"
        elif command in self.pick_place_commands:
            userdata.command = command
            return "pick_place"
        elif command in self.point_commands:
            userdata.command = command
            return "point"
        else:
            rospy.logwarn('Unknown command: %s', command)
            return "failure"


class DetectObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'transform_pose', 'failure', 'complete_failure'], input_keys=['command'])
        self.detection_op_topic = "object_detection/e_out"

        self.pub_obj_detection = rospy.Publisher("object_detection/e_in",
                                      String, queue_size=10, latch=True)
        self.counter = 3
        self.timeout = 5.0
        self.object_detected = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Detect')

        # command node
        rospy.logdebug("Publishing command: %s", userdata.command)
        self.pub_obj_detection.publish(userdata.command)

        # get response
        success = wait_for_success(self.detection_op_topic, self.timeout)
        if success:
            
            if userdata.command == 'detect_tomato':
                return 'success'
                
            if userdata.command == 'detect_truss':
                return 'transform_pose'
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
        self.counter = 1
        self.timeout = 60.0

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
        self.tf_op_topic = "pick_place/e_out"
        self.pub_pose_transform = rospy.Publisher("pick_place/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 40.0
        self.counter = 3
        self.tf_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Transform')

        # command node
        self.pub_pose_transform.publish("transform")

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
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], input_keys=['command'])
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
                return 'complete_failure'
            return 'failure'
            
            
class PickPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], input_keys=['command'])
        self.pick_place_op_topic = "pick_place/e_out"
        self.pub_pick_place = rospy.Publisher("pick_place/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 1
        self.mv_robot_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Pick Place')
        rospy.loginfo("Statemachine publishing command %s", userdata.command)

        # command node
        self.pub_pick_place.publish(userdata.command)

        # get response
        success = wait_for_success(self.pick_place_op_topic, self.timeout)

        # determine success
        if success:
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'


class Point(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], input_keys=['command'])
        self.point_op_topic = "point/e_out"
        self.pub_point = rospy.Publisher("point/e_in",
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 3
        self.mv_robot_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Point')

        # command node
        self.pub_point.publish("e_start")

        # get response
        success = wait_for_success(self.point_op_topic, self.timeout)

        # determine success
        if success:
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

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
                                            'pick_place': 'PickPlace',
                                            'point': 'Point',
                                            'failure': 'Idle'})

        smach.StateMachine.add('CalibrateRobot', CalibrateRobot(),
                           transitions={'success':'Idle',
                                        'failure': 'Idle',
                                        'complete_failure':'Idle'})
                                        
        smach.StateMachine.add('PickPlace', PickPlace(),
                           transitions={'success':'Idle',
                                        'failure': 'PickPlace',
                                        'complete_failure':'Idle'})
                                        
        smach.StateMachine.add('Point', Point(),
                           transitions={'success':'Idle',
                                        'failure': 'Point',
                                        'complete_failure':'Idle'})

        smach.StateMachine.add('DetectObject', DetectObject(),
                               transitions={'success': 'Idle',
                                           'transform_pose':'PoseTransform',
                                            'failure':'DetectObject',
                                            'complete_failure':'Idle'})

        smach.StateMachine.add('PoseTransform', PoseTransform(),
                               transitions={'success':'Idle',
                                            'failure':'DetectObject',
                                            'complete_failure':'Idle'})

        smach.StateMachine.add('MoveRobot', MoveRobot(),
                               transitions={'success':'Idle',
                                            'failure':'MoveRobot',
                                            'complete_failure': 'Idle'})

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
