#!/usr/bin/env python

import rospy
import smach
import smach_ros
import os

from std_msgs.msg import String, Bool
from flex_grasp.msg import FlexGraspErrorCodes
from flex_shared_resources.msg import GazeboInstruction

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log
from flex_shared_resources.utils.communication import Communication

outcomes = ['success', 'control_failure', 'planning_failure', 'state_failure', 'dynamixel_failure', 'severe_failure', 'failure']
NODE_NAME = 'pipeline'

def error_handling(result):

    if result == FlexGraspErrorCodes.SUCCESS:
        return 'success'
    elif result == FlexGraspErrorCodes.CONTROL_FAILED:
        return 'control_failure'# 'control_failure'
    elif result == FlexGraspErrorCodes.PLANNING_FAILED:
        return 'planning_failure'# 'planning_failure'
    elif result == FlexGraspErrorCodes.STATE_ERROR:
        return 'state_failure'
    elif result == FlexGraspErrorCodes.DYNAMIXEL_ERROR:
        return 'dynamixel_failure'
    elif result == FlexGraspErrorCodes.DYNAMIXEL_SEVERE_ERROR:
        return 'severe_failure'
    else:
        return 'failure' # 'unkown_failure'

class Initializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure', 'complete_failure'],
                             output_keys=['command', 'prev_command', 'mode'])
        timeout = 10.0
        
        # dict of nodes which need to be initialized!
        topic = {}
        topic['monitor_robot'] = 'monitor_robot'
        topic['transform_pose'] = 'transform_pose'
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

DEFAULT_SIMULATION = True
DEFAULT_PLAYBACK = False

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrate', 'detect', 'transform_pose', 'pick_place', 'move', 'spawn', 'failure'],
                             input_keys=['mode', 'command', 'prev_command'],
                             output_keys=['mode', 'command', 'prev_command'])
                             
        self.command_op_topic = 'pipeline_command'
        self.simulation = rospy.get_param("robot_sim", DEFAULT_SIMULATION)
        self.playback = rospy.get_param("playback", DEFAULT_PLAYBACK)
        self.experiment_id = None
        self.experiment_path = None

        self.experiment_id_publisher = rospy.Publisher('experiment_id', String, queue_size=1, latch=True)
        rospy.Subscriber("experiment", Bool, self.go_cb)
        rospy.Subscriber("experiment_pwd", String, self.experiment_pwd_cb)

        # commands
        self.detect_commands = ['detect_tomato', 'detect_truss', 'save_image']
        self.transform_commands = ['transform']
        self.calibrate_commands = ['calibrate', 'calibrate_height']
        self.move_commands = ['home', 'open', 'close', 'sleep', 'ready']
        self.pick_place_commands = ['pick', 'place', 'pick_place']
        self.spawn_commands = ['spawn_truss', 'set_pose_truss']
        self.experiment = False

    def experiment_pwd_cb(self, msg):
        """callback to update the data path"""
        new_path = msg.data
        if self.experiment_path != new_path:
            self.experiment_path = new_path
            rospy.logdebug("[{0}] Storing results in folder {1}".format(NODE_NAME, new_path))

    def go_cb(self, msg):
        self.experiment = msg.data
        rospy.logdebug("[{0}] experiment mode is set to %s".format(NODE_NAME, self.experiment))

    def execute(self, userdata):
        rospy.logdebug("[{0}] Executing state Idle".format(NODE_NAME))
        
        if self.experiment and (userdata.mode == 'free'):
            rospy.loginfo('[PIPELINE] Entering experiment mode!')
            userdata.mode = 'experiment'
            userdata.prev_command = None
        elif (userdata.mode == 'override') and (not self.experiment):
            userdata.mode = 'free'
        elif (not self.experiment) and userdata.mode == 'experiment':
            userdata.mode = 'free'

        if userdata.mode == 'experiment':
            if (userdata.prev_command is None or userdata.prev_command == 'reset') and self.simulation:
                userdata.command = 'spawn_truss'
            elif userdata.prev_command is None or userdata.prev_command == 'reset' or userdata.prev_command == 'spawn_truss' or userdata.prev_command == 'set_pose_truss':
                userdata.command = 'detect_truss'
            elif userdata.prev_command == 'detect_truss':
                userdata.command = 'transform'
            elif userdata.prev_command == 'transform':
                userdata.command = 'pick'
            elif userdata.prev_command == 'pick':
                userdata.command = 'place'
            elif userdata.prev_command == 'place' and self.simulation:
                userdata.command = 'set_pose_truss'
            elif userdata.prev_command == 'place' and not self.simulation:
                userdata.command = 'detect_truss'
            else:
                rospy.logwarn("[{0}] do not know what to do with previous command {1}".format(NODE_NAME, userdata.prev_command))
        elif userdata.prev_command == 'detect_truss':
            userdata.command = 'transform'
        else:
            userdata.command = rospy.wait_for_message(self.command_op_topic, String).data    

        if userdata.command in self.transform_commands:
            return 'transform_pose'
        elif userdata.command in self.detect_commands:
            if self.playback:
                self.experiment_id = self.get_next_experiment_id()
            else:
                self.experiment_id = self.get_new_experiment_id()
            rospy.logdebug("[{0}] Publishing experiment id {1}".format(NODE_NAME, self.experiment_id))
            self.experiment_id_publisher.publish(self.experiment_id)
            return 'detect'
        elif userdata.command in self.move_commands:
            return 'move'
        elif userdata.command in self.calibrate_commands:
            return 'calibrate'
        elif userdata.command in self.pick_place_commands:
            return 'pick_place'
        elif userdata.command in self.spawn_commands:
            return 'spawn'
        else:
            rospy.logwarn('Unknown command: %s', userdata.command)
            userdata.command = None
            userdata.mode = 'free'
            return 'failure'

    def get_next_experiment_id(self):
        """increment the id by one if it exsists"""
        if self.experiment_id is None:
            id_int = 1
        else:
            id_int = int(self.experiment_id) + 1
        return str(id_int).zfill(3)

    def get_new_experiment_id(self):
        """returns the lowest available id in the given path"""

        if not os.path.isdir(self.experiment_path):
            new_id_int = 1
        else:
            contents = os.listdir(self.experiment_path)
            if len(contents) == 0:
                new_id_int = 1
            else:
                contents.sort()
                file_name = contents[-1]
                file_id = file_name[:3]
                new_id_int = int(file_id) + 1

        return str(new_id_int).zfill(3)

class CalibrateRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
                             
        topic = 'calibration_eye_on_base/calibrate'
        timeout = 60.0
        self.communication = Communication(topic, timeout = timeout)        

    def execute(self, userdata):
        rospy.logdebug('Executing state Calibrate')
    
        # command node
        result = self.communication.wait_for_result(userdata.command)
        userdata.prev_command = userdata.command
        userdata.command = None        
        return error_handling(result)

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


class SpawnObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'complete_failure'],
                             input_keys=['mode', 'command', 'prev_command'],
                             output_keys=['mode', 'command', 'prev_command'])
        topic = 'gazebo_interface'
        timeout = 2.0
        self.communication = Communication(topic, timeout=timeout, msg_type=GazeboInstruction)
        self.counter = 3

    def execute(self, userdata):
        rospy.logdebug('Executing state SpawnObject')

        # command node
        if userdata.command == 'spawn_truss':
            msg = GazeboInstruction(command=GazeboInstruction.SPAWN, model_type='3d')
        elif userdata.command == 'set_pose_truss':
            msg = GazeboInstruction(command=GazeboInstruction.SETPOSE)
        else:
            rospy.logerr("Unknown command %s", userdata.command)

        result = self.communication.wait_for_result(msg)

        if result == FlexGraspErrorCodes.SUCCESS:
            userdata.prev_command = userdata.command
            userdata.command = None
            return 'success'
        else:
            self.counter = self.counter - 1
            if self.counter <= 0:
                return 'complete_failure'
            return 'failure'


class PoseTransform(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'], 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
                     
                     
        topic = 'transform_pose'
        timeout = 40.0
        self.communication = Communication(topic, timeout = timeout)


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
            return 'failure'

class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes, 
                             input_keys=['mode', 'command', 'prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])

        topic = 'move_robot'
        timeout = 30.0
        self.communication = Communication(topic, timeout=timeout)

    def execute(self, userdata):
        
        rospy.logdebug('Executing state Move Robot')
        result = self.communication.wait_for_result(userdata.command)

        userdata.prev_command = userdata.command
        userdata.command = None        
        return error_handling(result)
            
            
class PickPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes, 
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
            outcome = error_handling(result)
            if outcome != 'success':
                return outcome
        
        # determine success
        userdata.prev_command = userdata.command
        userdata.command = None 
        return outcome

class ResetArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes, 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
        topic = 'move_robot'
        timeout = 30.0
        self.max_attempts = 3
        self.communication = Communication(topic, timeout = timeout)
    
    def execute(self, userdata):
            
        for attempt_remaining in reversed(range(self.max_attempts)):
            rospy.loginfo("[PIPELINE] Opening end effector")
            result = self.communication.wait_for_result('open')
    
            if result == FlexGraspErrorCodes.SUCCESS:       
                rospy.loginfo("[PIPELINE] Homeing manipulator")
                result = self.communication.wait_for_result('home')
            
            if result == FlexGraspErrorCodes.SUCCESS:
                userdata.command = None
                userdata.prev_command = 'reset'  
                return 'success'
                
            flex_grasp_error_log(result, "PIPELINE")
            rospy.logwarn("[PIPELINE] retry resetarm: %s attempts remaining", attempt_remaining)

        return error_handling(result)
        
class ResetDynamixel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
        timeout = 30.0
        self.counter = 1
        self.move_robot_communication = Communication('move_robot', timeout = timeout)
        self.monitor_robot_communication = Communication('monitor_robot', timeout = timeout)        
        
    def execute(self, userdata):
        rospy.loginfo("[PIPELINE] Forceing robot")
        result = self.move_robot_communication.wait_for_result('force_robot')   
            
        rospy.loginfo("[PIPELINE] Opening end effector")
        result = self.move_robot_communication.wait_for_result('open')

        rospy.loginfo("[PIPELINE] Sleep manipulator")
        result = self.move_robot_communication.wait_for_result('sleep')        

        result = self.monitor_robot_communication.wait_for_result('reboot')
        
        result = self.move_robot_communication.wait_for_result('do_not_force_robot')
        
        # check if error is fixed
        result = self.monitor_robot_communication.wait_for_result('monitor_robot')
        if result != FlexGraspErrorCodes.SUCCESS:
            flex_grasp_error_log(result, "PIPELINE")
            return 'failure'
        
        result = self.move_robot_communication.wait_for_result('sleep') 
        result = self.move_robot_communication.wait_for_result('open')
        result = self.move_robot_communication.wait_for_result('home')            
        
        userdata.command = None
        userdata.prev_command = 'reset'            
        return 'success'
  
class RestartMode(smach.State):    
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
    def execute(self, userdata):
        userdata.command = None
        userdata.prev_command = 'reset'
        return 'success'            
            
class StopMode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], 
                             input_keys=['mode', 'command','prev_command'], 
                             output_keys=['mode', 'command', 'prev_command'])
        
    def execute(self, userdata):
        userdata.mode = 'override'# 'free'
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
                                            'spawn': 'SpawnObject',
                                            'failure': 'Idle'})

        smach.StateMachine.add('CalibrateRobot', CalibrateRobot(),
                           transitions={'success':'Idle',
                                            'control_failure': 'ResetArm',
                                            'planning_failure': 'StopMode',
                                            'state_failure': 'RestartMode',
                                            'dynamixel_failure' : 'ResetDynamixel',
                                            'severe_failure' : 'total_failure',
                                            'failure':'StopMode'})
                                        
        smach.StateMachine.add('PickPlace', PickPlace(),
                           transitions={'success':'Idle',
                                            'control_failure': 'ResetArm',
                                            'planning_failure': 'StopMode',
                                            'state_failure': 'RestartMode',
                                            'dynamixel_failure': 'ResetDynamixel',
                                            'severe_failure': 'total_failure',
                                            'failure': 'StopMode'})

        smach.StateMachine.add('SpawnObject', SpawnObject(),
                               transitions={'success': 'Idle',
                                            'failure': 'SpawnObject',
                                            'complete_failure': 'StopMode'})

        smach.StateMachine.add('DetectObject', DetectObject(),
                               transitions={'success': 'Idle',
                                            'failure': 'DetectObject',
                                            'complete_failure': 'StopMode'})

        smach.StateMachine.add('PoseTransform', PoseTransform(),
                               transitions={'success': 'Idle',
                                            'failure': 'RestartMode'})

        smach.StateMachine.add('MoveRobot', MoveRobot(),
                               transitions={'success':'Idle',
                                            'control_failure': 'ResetArm',
                                            'planning_failure': 'StopMode',
                                            'state_failure': 'RestartMode',
                                            'dynamixel_failure' : 'ResetDynamixel',
                                            'severe_failure' : 'total_failure',
                                            'failure':'StopMode'})
                                           
        smach.StateMachine.add('ResetArm', ResetArm(),
                               transitions={'success':'Idle',
                                            'control_failure': 'StopMode',
                                            'planning_failure': 'StopMode',
                                            'state_failure': 'StopMode',
                                            'dynamixel_failure' : 'ResetDynamixel',
                                            'severe_failure' : 'total_failure',
                                            'failure':'StopMode'})
 
        smach.StateMachine.add('ResetDynamixel', ResetDynamixel(),
                               transitions={'success':'Idle',
                                            'failure':'StopMode'})
                                           
        smach.StateMachine.add('StopMode', StopMode(),
                               transitions={'success':'Idle',
                                            'failure':'total_failure'})

        smach.StateMachine.add('RestartMode', RestartMode(),
                               transitions={'success':'Idle',
                                            'failure':'total_failure'})

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
