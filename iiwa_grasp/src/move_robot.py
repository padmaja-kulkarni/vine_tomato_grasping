#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
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

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
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
                        anonymous=True)
        
        rospy.Subscriber("endEffector", geometry_msgs.msg.PoseStamped, CBfunction)
        
    def CBfunction(self, msg):
        if self.robot_goal_pose is None:
            self.robot_goal_pose = msg
            
    def initialise_robot(self):
        ## First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)
        
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()
        
        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        
        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        
        
        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def go_to_pose_goal(self):
        ## plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        self.group.set_pose_target(self.robot_pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()
        # For testing:
        current_pose = self.group.get_current_pose().pose
        return all_close(self.robot_goal_pose, current_pose, 0.02)
    
    
    def command_robot(self):
        success = None
        if self.robot_goal_pose is not None:
            success = MoveRobot.go_to_pose_goal()
            if success:
                self.robot_goal_pose is None
            else:
                print("Warning: Robot Position is not within Tolerance of Goal Position")
    
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
            
            
        
        


