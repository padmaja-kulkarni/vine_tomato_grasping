#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes

def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'                                      
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'




def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveRobot(object):
    """MoveRobot"""
    def __init__(self):
        super(MoveRobot, self).__init__()
        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('Move_Robot',
                        anonymous=True,
                        log_level=rospy.DEBUG)
        
        
        self.initialise_robot()
        self.initialise_enviroment()
        
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        
        callback_lambda = lambda msg: CBfunction(msg, self)
        
        # if inbound connection is of wrong topic type, an warning will be thrown
        rospy.Subscriber("endEffectorPose", PoseStamped, callback_lambda)
        
        
        print "kown constaints: ", self.group.get_known_constraints()
        
        def CBfunction(msg, self):
            if self.robot_goal_pose is None:
                self.robot_goal_pose = msg
                rospy.logdebug("Received new message")
                
            
    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")
        
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()
        
        # interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        group_name = rospy.get_param('move_group')
        group = moveit_commander.MoveGroupCommander(group_name)

        # rospy.sleep(10)required?

        self.robot = robot 
        self.group = group
        self.robot_goal_pose = None
        self.scene = scene


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name = "box"):
        rospy.logdebug("waitng for state update...")
        ## To ensure that the updates are made, we wait until we see the 
        ## changes reflected in the ``get_known_object_names()`` and 
        ## ``get_known_object_names()`` lists.
        start = rospy.get_time()
        seconds = rospy.get_time()
        
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects()
            is_attached = len(attached_objects.keys()) > 0
            
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()
            
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
    
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
    
        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4, box_name = 'box'):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        
        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = PoseStamped()
        box_pose.header.frame_id =  self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.
        box_pose.pose.position.y = 0.
        box_pose.pose.position.z = 0.
        
        
        scene.add_box(box_name, box_pose, size=(1, 1, 1))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name = box_name)
    
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.
        
        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.
        
        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        
        group_name = rospy.get_param('move_group')
        request.ik_request.group_name = group_name
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = moveit_error_string(response.error_code.val)
        print error_str
        success = error_str == 'SUCCESS'
        if not success:
            return False
        # joint_state = response.solution.joint_state
        # for name, position in zip(joint_state.name, joint_state.position):
        #     if name in ArmJoints.names():
        #         rospy.loginfo('{}: {}'.format(name, position))
        return True
    
    def initialise_enviroment(self):     
        rospy.logdebug("===INITIALIZING ENVIROMENT====")
        if self.add_box(box_name = 'table'):
            rospy.logdebug("Succesfully added table")
        else:
            rospy.logwarn("Unable to add table")

        
        rospy.logdebug( "Known objects: %s", self.scene.get_known_object_names())
    def go_to_pose_goal(self):
        ## Planning to a Pose Goal
        
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        
        if self.compute_ik(self.robot_goal_pose):
            rospy.loginfo("Goal pose is reacchable.")
            
            self.group.set_pose_target(self.robot_goal_pose.pose)
            # self.group.set_start_state_to_current_state()
            
            ## Now, we call the planner to compute the plan
            plan = self.group.plan()
            
            # Now execute the plan
            self.group.execute(plan, wait=True)
            
        else:
            rospy.logwarn("Goal pose is not reachable!")
            
        
        
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()
        self.robot_goal_pose = None
        # For testing:
        current_pose = self.group.get_current_pose()
        
        return all_close(self.robot_goal_pose, current_pose, 0.02)
    
    
    def command_robot(self):
        success = None
        if self.robot_goal_pose is not None:
            success = self.go_to_pose_goal()            
            if not(success):
                rospy.logwarn("Goal Tolerance Violated")
    
def main():
    try:
        moverobot = MoveRobot()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            moverobot.command_robot()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == '__main__':
    main()
            
            
        
        


