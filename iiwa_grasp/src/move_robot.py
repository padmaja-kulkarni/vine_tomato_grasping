#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list

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
        
        self.initialise_robot()
        
        rospy.init_node('Move_Robot',
                        anonymous=True,
                        log_level=rospy.DEBUG)
        
        callback_lambda = lambda msg: CBfunction(msg, self)
        
        # if inbound connection is of wrong topic type, an warning will be thrown
        rospy.Subscriber("endEffectorPose", PoseStamped, callback_lambda)
        
        self.pub = rospy.Publisher("moveRobotSuccess", Bool, queue_size=10, latch=True)
        
        
        
        def CBfunction(msg, self):
            if self.robot_goal_pose is None:
                self.robot_goal_pose = msg
                rospy.logdebug("Received new message")
                
            
    def initialise_robot(self):
        
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        group_name = rospy.get_param('move_group')
        group = moveit_commander.MoveGroupCommander(group_name)

        self.robot = robot
        self.group = group
        self.robot_goal_pose = None
        
    def go_to_pose_goal(self):
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        self.group.set_pose_target(self.robot_goal_pose.pose)
        self.group.set_start_state_to_current_state()
        
        ## Now, we call the planner to compute the plan
        plan = self.group.plan()
        
        # Now execute the plan
        self.group.execute(plan, wait=True)
        
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
            if success:
                self.pub.publish(True)
            else:
                self.pub.publish(False)
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
            
            
        
        


