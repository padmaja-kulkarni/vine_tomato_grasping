#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInterface(object):
  """MoveGroupPythonInterface"""
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python',
                    anonymous=True)

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
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, pose_goal):

    ## plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    
    self.group.set_pose_target(pose_goal)
    current_pose = self.group.get_current_pose().pose
    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.group.clear_pose_targets()

    # For testing:
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.02)


def main():
  try:
    print "============ Press `Enter` to begin"
    raw_input()
    MoveGroup = MoveGroupPythonInterface()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = -0.310
    pose_goal.orientation.y = 0.000
    pose_goal.orientation.z = 0.001
    pose_goal.orientation.w = 0.951
    pose_goal.position.x = -0.014
    pose_goal.position.y = 0.262
    pose_goal.position.z = 1.127

    MoveGroup.go_to_pose_goal(pose_goal)

    print "============ Go to pose complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
    
if __name__ == '__main__':
  main()
