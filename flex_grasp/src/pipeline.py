#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool
from func.ros_utils import wait_for_success, wait_for_result
from flex_grasp.msg import FlexGraspErrorCodes
from func.flex_grasp_error import flex_grasp_error_log
from communication import Communication


class Initializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'],
                             output_keys=['command', 'prev_command', 'mode'])
        timeout = 10.0
        
        # dict of nodes which need to be initialized!
        topic = {}
        topic['object_detection'] = 'object_detection'
        topic['pick_place'] = 'pick_place'
        topic['move_robot'] = 'move_robot'
        topic['calibrate'] = 'calibration_eye_on_base/calibrate'

        # create for each node a communication channel
        communication = {}
        for key in topic:
            communication[key] = Communication(topic[key], timeout = timeout)  

        self.communication = communication

    def execute(self, userdata):
        rospy.logdebug('Executing state Initializing')
        
        # command all nodes via the initialized communication channels to initialize, wiat for their response
        result = {}
        for key in self.communication:
            result[key] = self.communication[key].wait_for_result('e_init')
        
        all_initialized = True
        for key in result:
            if result[key] != FlexGraspErrorCodes.SUCCESS:
                rospy.logwarn("[PIPELINE] Failed to initialize %s", key)
                all_initialized = False
        
        if all_initialized:
            userdata.mode = 'free'
            userdata.command = None
            userdata.prev_command = 'initialize'
            return 'success'
        else:
            return 'failure'


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrate', 'detect', 'transform_pose', 'pick_place', 'move', 'point', 'failure'], 
                             input_keys=['mode', 'command', 'prev_command'],
                             output_keys=['mode', 'command', 'prev_command'])
                             
        self.command_op_topic = 'pipeline_command'
        rospy.Subscriber("experiment", Bool, self.go_cb)

        self.detect_commands =  ['detect_tomato', 'detect_truss', 'save_image']
        self.transform_commands = ['transform']
        self.calibrate_commands =  ['calibrate']
        self.move_commands =  ['home', 'open', 'close', 'sleep']
        self.pick_place_commands = ['pick', 'place', 'pick_place']
        self.point_commands = ['point']
        self.experiment = False

    def go_cb(self, msg):
        self.experiment = msg.data
        rospy.logdebug("[PIPELINE] experiment: %s", self.experiment)

    def execute(self, userdata):
        rospy.logdebug('Executing state Idle')
        
        if self.experiment and (userdata.mode != 'experiment'):
            rospy.loginfo('[PIPELINE] Entering experiment mode!')
            userdata.mode = 'experiment'
            userdata.prev_command == None
        elif (not self.experiment) and userdata.mode == 'experiment':
            userdata.mode = 'free'
            
                
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
        else:
            rospy.logwarn('Unknown command: %s', userdata.command)
            return 'failure'

class CalibrateRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
                             
        topic = 'calibration_eye_on_base/calibrate'
        timeout = 60.0
        self.communication = Communication(topic, timeout = timeout)        
        
        self.counter = 1
        

    def execute(self, userdata):
        rospy.logdebug('Executing state Calibrate')
    
        # command node
        result = self.communication.wait_for_result(userdata.command)

        if result == FlexGraspErrorCodes.SUCCESS:
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
        topic = 'object_detection'
        timeout = 25.0
        self.communication = Communication(topic, timeout = timeout) 
        self.counter = 3

    def execute(self, userdata):
        rospy.logdebug('Executing state Detect')
        
        # This ensures the robot is not in the way of the camera, even with delay.
        rospy.sleep(1.0)
        
        # command node
        result = self.communication.wait_for_result(userdata.command)
        
        if result == FlexGraspErrorCodes.SUCCESS:
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
                     
                     
        topic = 'pick_place'
        timeout = 40.0
        self.communication = Communication(topic, timeout = timeout)
        
        self.counter = 3

    def execute(self, userdata):
        rospy.logdebug('Executing state Transform')

        # get response
        result = self.communication.wait_for_result(userdata.command)

        # determine success
        if result == FlexGraspErrorCodes.SUCCESS:
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

        topic = 'move_robot'
        timeout = 30.0
        self.counter = 1
        self.communication = Communication(topic, timeout = timeout)

    def execute(self, userdata):
        
        rospy.logdebug('Executing state Move Robot')
        result = self.communication.wait_for_result(userdata.command)

        # determine success
        if result == FlexGraspErrorCodes.SUCCESS:
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
        topic = 'pick_place'
        timeout = 30.0
        self.communication = Communication(topic, timeout = timeout)

    def execute(self, userdata):
        rospy.logdebug('Executing state Pick Place')
        rospy.logdebug('Statemachine publishing command %s', userdata.command)

        if userdata.command == "pick_place":
            commands = ["pick", "place"]
        else:
            commands = [userdata.command]
        
        for command in commands:
            
            result = self.communication.wait_for_result(command)
            flex_grasp_error_log(result)
            
            if result == FlexGraspErrorCodes.SUCCESS:
                pass
            elif result == FlexGraspErrorCodes.CONTROL_FAILED:
                return 'failure'# 'control_failure'
            elif result == FlexGraspErrorCodes.PLANNING_FAILED:
                return 'failure'# 'planning_failure'
            elif result == FlexGraspErrorCodes.STATE_ERROR:
                return 'failure'
            else:
                return 'failure' # 'unkown_failure'
        
        # determine success
        if result == FlexGraspErrorCodes.SUCCESS:
            userdata.prev_command = userdata.command
            userdata.command = None
            return 'success' 

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
        
        topic = 'move_robot'
        timeout = 30.0
        self.counter = 1
        self.communication = Communication(topic, timeout = timeout)
        
        
    def execute(self, userdata):
        
        pick_place_commands = ['pick', 'place', 'pick_place']
        if userdata.command in pick_place_commands:
            
            rospy.loginfo("Opening end effector")
            result = self.communication.wait_for_result('open')

            if result != FlexGraspErrorCodes.SUCCESS:
                flex_grasp_error_log(result, "PIPELINE")
                return 'failure'
                
            rospy.loginfo("Homeing manipulator")
            result = self.communication.wait_for_result('home')
            
            if result != FlexGraspErrorCodes.SUCCESS:
                flex_grasp_error_log(result, "PIPELINE")
                return 'failure'
            
            # this is done since the end-effector might hace preempted in an open state, this wont be noted otherwise...
            rospy.loginfo("Closing end effector")
            result = self.communication.wait_for_result('close')

            if result != FlexGraspErrorCodes.SUCCESS:
                flex_grasp_error_log(result, "PIPELINE")
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
                                        'failure': 'Reset',
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
