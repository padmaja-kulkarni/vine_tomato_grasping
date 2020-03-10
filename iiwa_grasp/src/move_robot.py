#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# custom functions
from func.moveit_error_string import moveit_error_string
from func.all_close import all_close

# enum
from RobotState import RobotState  


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
        
        while rospy.get_time() < 0.1:
            pass
        
        self.initialise_robot()
        self.initialise_enviroment()
        
        
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        
        callback_lambda = lambda msg: CBfunction(msg, self)
        
        # if inbound connection is of wrong topic type, an warning will be thrown
        rospy.Subscriber("endEffectorPose", PoseStamped, callback_lambda)
        # self.pub = rospy.Publisher('robotState', int, queue_size=5, latch=True)
        # self.pub.publish(self.state)
        
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
        group_name = rospy.get_param('move_group_name')
        group = moveit_commander.MoveGroupCommander(group_name)

        # rospy.sleep(10)required?

        self.robot = robot 
        self.group = group
        self.robot_goal_pose = None
        self.scene = scene
        self.state = RobotState.INITIALIZING
        self.box_name = 'table'
        
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4, box_name= ''):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
    
        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects()
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                print "Known objects: ", scene.get_known_object_names()
                return True
            
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
    
        # If we exited the while loop without returning then we timed out
        print "Known objects: ", scene.get_known_object_names()
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4, box_name = ''):
        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = PoseStamped()
        box_pose.header.frame_id =  self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.y = 1.0
        
        self.scene.add_box(box_name, box_pose, size=(1, 1, 0.1))
        
        print "box frame: " ,box_pose.header.frame_id
        
        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
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
        
        group_name = rospy.get_param('move_group_name')
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
        rospy.sleep(5)
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
            
            
        
        


