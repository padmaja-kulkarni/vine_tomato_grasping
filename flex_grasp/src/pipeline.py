#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool
from func.ros_utils import wait_for_success, wait_for_result
from flex_grasp.msg import MoveRobotResult, PickPlaceResult

class Initializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'],
                             output_keys=['command', 'prev_command', 'mode'])
        self.timeout = 10.0

        self.pub_object_detection = rospy.Publisher('object_detection/e_in',
                                      String, queue_size=10, latch=True)
        self.pub_pose_transform = rospy.Publisher('pick_place/e_in',
                                    String, queue_size=10, latch=True)
        self.pub_move_robot = rospy.Publisher('move_robot/e_in',
                                      String, queue_size=10, latch=True)
        self.pub_calibrate = rospy.Publisher('calibration_eye_on_base/calibrate/e_in',
                              String, queue_size=10, latch=True)

    def execute(self, userdata):
        rospy.logdebug('Executing state Initializing')

        init_object_detection = self.is_initialized('object_detection/e_out', self.pub_object_detection)
        init_pose_transform = self.is_initialized('pick_place/e_out', self.pub_pose_transform, msg_type = PickPlaceResult)
        init_move_robot = self.is_initialized('move_robot/e_out', self.pub_move_robot, msg_type = MoveRobotResult)
        init_calibrate = self.is_initialized('calibration_eye_on_base/calibrate/e_out', self.pub_calibrate)

        userdata.mode = 'free'
        userdata.command = None
        userdata.prev_command = 'initialize'

        if init_object_detection & init_pose_transform & init_move_robot & init_calibrate:
            return 'success'
        else:
            rospy.logwarn("Failed to initialize")
            rospy.loginfo("init_object_detection %s", init_object_detection)
            rospy.loginfo("init_pose_transform %s", init_pose_transform)
            rospy.loginfo("init_move_robot %s", init_move_robot)
            rospy.loginfo("init_calibrate %s", init_calibrate)
            return 'failure'

    def is_initialized(self, topic_out, topic_in, msg_type = None):
        request = String()
        request.data = 'e_init'
        topic_in.publish(request)
        
        if msg_type is not None:
            return wait_for_result(topic_out, self.timeout, msg_type)
        else:
            return wait_for_success(topic_out, self.timeout)


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrate', 'detect', 'transform_pose', 'pick_place', 'move', 'point', 'failure'], 
                             input_keys=['mode', 'command', 'prev_command'],
                             output_keys=['mode', 'command', 'prev_command'])
                             
        self.command_op_topic = 'pipelineState'

        self.detect_commands =  ['detect_tomato', 'detect_truss', 'save_image']
        self.transform_commands = ['transform']
        self.calibrate_commands =  ['calibrate']
        self.move_commands =  ['home', 'open', 'close', 'sleep']
        self.pick_place_commands = ['pick', 'place', 'pick_place']
        self.point_commands = ['point']
        self.reset_commands = ['reset', 'hard_reset']
        
        self.pub_is_idle = rospy.Publisher('pipeline/is_idle',
                              Bool, queue_size=10, latch=True)

    def execute(self, userdata):
        rospy.logdebug('Executing state Idle')
    
        if userdata.mode == 'experiment':
            if userdata.prev_command == None:
                userdata.command = 'calibrate'
            elif userdata.prev_command == 'calibrate' or userdata.prev_command == 'reset':
                userdata.command = 'detect_truss'
            elif userdata.prev_command == 'detect_truss':
                userdata.command = 'transform'
            elif userdata.prev_command == 'transform':
                userdata.command = 'pick'
            elif userdata.prev_command == 'pick':
                userdata.command = 'place'
            elif userdata.prev_command == 'place':
                userdata.command = 'detect_truss'
        elif userdata.prev_command == 'detect_truss':
            userdata.command = 'transform'
        else:
            userdata.command = rospy.wait_for_message(self.command_op_topic, String).data    
            
        if userdata.command in self.transform_commands:
            return 'transform_pose'
        elif userdata.command in self.detect_commands:         
            return 'detect'
        elif userdata.command in self.move_commands:
            return 'move'
        elif userdata.command in self.calibrate_commands:
            return 'calibrate'
        elif userdata.command in self.pick_place_commands:
            return 'pick_place'
        elif userdata.command in self.point_commands:
            return 'point'
        elif userdata.command == 'experiment':
            rospy.loginfo('Entering experiment mode!')
            userdata.mode = 'experiment'     
            userdata.command = 'calibrate'
            return 'calibrate'
        else:
            rospy.logwarn('Unknown command: %s', userdata.command)
            return 'failure'

class CalibrateRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
                             
        self.calibrate_topic = 'calibration_eye_on_base/calibrate/e_out'

        self.pub_calibrate = rospy.Publisher('calibration_eye_on_base/calibrate/e_in',
                                      String, queue_size=10, latch=True)
        self.counter = 1
        self.timeout = 60.0

    def execute(self, userdata):
        rospy.logdebug('Executing state Calibrate')
    
        # command node
        self.pub_calibrate.publish(userdata.command)

        # get response
        success = wait_for_success(self.calibrate_topic, self.timeout)

        if success:
            userdata.prev_command = userdata.command
            userdata.command = None
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class DetectObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        self.detection_op_topic = 'object_detection/e_out'

        self.pub_obj_detection = rospy.Publisher('object_detection/e_in',
                                      String, queue_size=10, latch=True)
        self.counter = 3
        self.timeout = 25.0
        self.object_detected = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Detect')

        # command node
        self.pub_obj_detection.publish(userdata.command)

        # get response
        success = wait_for_success(self.detection_op_topic, self.timeout)
        if success:
            
            if userdata.command == 'detect_tomato' or userdata.command == 'save_image':
                userdata.prev_command = userdata.command
                userdata.command = None
                return 'success'
                
            if userdata.command == 'detect_truss':
                userdata.prev_command = userdata.command
                userdata.command = None
                return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                return 'complete_failure'
            return 'failure'

class PoseTransform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure','complete_failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
                             
        self.tf_op_topic = 'pick_place/e_out'
        self.pub_pose_transform = rospy.Publisher('pick_place/e_in',
                                      String, queue_size=10, latch=True)
        self.timeout = 40.0
        self.counter = 3
        self.tf_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Transform')

        # command node
        self.pub_pose_transform.publish(userdata.command)

        # get response
        success = wait_for_result(self.tf_op_topic, self.timeout, PickPlaceResult)

        # determine success
        if success:
            userdata.prev_command = userdata.command
            userdata.command = None
            return 'success'
        else:
            self.counter = self.counter -1
            if self.counter <=0:
                userdata.prev_command = userdata.command
                return 'complete_failure'
            return 'failure'

class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        self.mv_robot_op_topic = 'move_robot/e_out'
        self.pub_move_robot = rospy.Publisher('move_robot/e_in',
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 3

    def execute(self, userdata):
        rospy.logdebug('Executing state Move Robot')

        # command node
        self.pub_move_robot.publish(userdata.command)

        # get response
        success = wait_for_result(self.mv_robot_op_topic, self.timeout, MoveRobotResult)

        # determine success
        if success:
            userdata.command = None
            userdata.prev_command = userdata.command
            return 'success'       
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                userdata.prev_command = userdata.command
                return 'complete_failure'
            return 'failure'
            
            
class PickPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        self.pick_place_op_topic = 'pick_place/e_out'
        self.pub_pick_place = rospy.Publisher('pick_place/e_in',
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 1

    def execute(self, userdata):
        rospy.logdebug('Executing state Pick Place')
        rospy.loginfo('Statemachine publishing command %s', userdata.command)

        # command node
        self.pub_pick_place.publish(userdata.command)

        # get response
        success = wait_for_result(self.pick_place_op_topic, self.timeout, PickPlaceResult)

        # determine success
        if success:
            userdata.prev_command = userdata.command
            userdata.command = None
            return 'success'                
        else:
            self.counter = self.counter - 1
            if self.counter <=0:
                userdata.prev_command = userdata.command
                return 'complete_failure'
            return 'failure'


class Point(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'], input_keys=['command'])
        self.point_op_topic = 'point/e_out'
        self.pub_point = rospy.Publisher('point/e_in',
                                      String, queue_size=10, latch=True)
        self.timeout = 30.0
        self.counter = 3
        self.mv_robot_result = String()

    def execute(self, userdata):
        rospy.logdebug('Executing state Point')

        # command node
        self.pub_point.publish('e_start')

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

class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
        self.pub_move_robot = rospy.Publisher('move_robot/e_in',
                                      String, queue_size=10, latch=True)        
                                     
        self.mv_robot_op_topic = 'move_robot/e_out'
        
    def execute(self, userdata):
        
        pick_place_commands = ['pick', 'place', 'pick_place']
        if userdata.prev_command in pick_place_commands:
            
            rospy.loginfo("Opening EE")
            self.pub_move_robot.publish('open')
            success = wait_for_result(self.mv_robot_op_topic, 5, MoveRobotResult)

            if not success:
                return 'failure'
                
            rospy.loginfo("Homeing manipulator")
            self.pub_move_robot.publish('home')
            success = wait_for_result(self.mv_robot_op_topic, 10, MoveRobotResult)
            
            if not success:
                return 'failure'
            
            userdata.command = None
            userdata.prev_command = 'reset'            
            return 'success'
            
        elif userdata.prev_command == 'detect_truss':
            return 'failure'
            
        else:
            userdata.command = None
            userdata.prev_command = 'reset'
            return 'success'
            
            
class HardReset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
    def execute(self, userdata):
        userdata.mode = 'free'
        userdata.command = None
        userdata.prev_command = 'hard_reset'
        return 'success'

# main
def main():
    debug_mode = rospy.get_param('pipeline/debug')

    if debug_mode:
        log_level = rospy.DEBUG
        rospy.loginfo('[PIPELINE] Luanching pipeline node in debug mode')
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
                               transitions={'calibrate': 'CalibrateRobot',
                                            'detect':'DetectObject',                                            
                                            'transform_pose':'PoseTransform',
                                            'pick_place': 'PickPlace',
                                            'move': 'MoveRobot',
                                            'point': 'Point',
                                            'failure': 'Idle'})

        smach.StateMachine.add('CalibrateRobot', CalibrateRobot(),
                           transitions={'success':'Idle',
                                        'failure': 'Idle',
                                        'complete_failure':'Reset'})
                                        
        smach.StateMachine.add('PickPlace', PickPlace(),
                           transitions={'success':'Idle',
                                        'failure': 'PickPlace',
                                        'complete_failure':'Reset'})
                                        
        smach.StateMachine.add('Point', Point(),
                           transitions={'success':'Idle',
                                        'failure': 'Point',
                                        'complete_failure':'Reset'})

        smach.StateMachine.add('DetectObject', DetectObject(),
                               transitions={'success': 'Idle',
                                            'failure':'DetectObject',
                                            'complete_failure':'Reset'})

        smach.StateMachine.add('PoseTransform', PoseTransform(),
                               transitions={'success':'Idle',
                                            'failure':'DetectObject',
                                            'complete_failure':'Reset'})

        smach.StateMachine.add('MoveRobot', MoveRobot(),
                               transitions={'success':'Idle',
                                            'failure':'MoveRobot',
                                            'complete_failure': 'Reset'})
                                           
        smach.StateMachine.add('Reset', Reset(),
                               transitions={'success':'Idle',
                                            'failure':'HardReset'})

        smach.StateMachine.add('HardReset', Reset(),
                               transitions={'success':'Idle',
                                            'failure':'total_failure'})

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
