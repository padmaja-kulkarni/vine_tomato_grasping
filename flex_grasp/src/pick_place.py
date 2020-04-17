#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

import sys
import rospy
import moveit_commander
import math as m
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation, Grasp, GripperTranslation

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from std_msgs.msg       import Float64

# custom functions
from func.all_close import all_close

class Pick_Place(object):
    """Pick_Place"""
    def __init__(self):
        super(Pick_Place, self).__init__()

        rospy.init_node("Pick_Place",
                        anonymous=True,
                        log_level=rospy.DEBUG)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass

        self.initialise_robot()
        self.initialise_enviroment()


        self.GRIPPER_EFFORT = [100.0]

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.state = "idle"
        self.prev_state = None
        self.goal_state = None

        # Subscribers
        rospy.Subscriber("endEffectorPose", PoseStamped, self.ee_pose_cb)
        rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers
        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=True)

    def ee_distance_cb(self, msg):
        dist = msg.data
        if self.EE_GRASP is None:
            dist = msg.data
            self.EE_GRASP = add_list(self.EE_CLOSED, [dist/2, -dist/2])
            rospy.logdebug("[PICK PLACE] Received a new end effector distance message")
            rospy.logdebug("[PICK PLACE] New end effector grasp pose: %s", self.EE_GRASP)

    def ee_pose_cb(self, msg):
        if self.cage_pose is None:
            self.cage_pose = msg
            rospy.logdebug("[PICK PLACE] Received new move robot pose message")

    def e_in_cb(self, msg):
        if self.goal_state is None:
            self.goal_state = msg.data
            rospy.logdebug("[PICK PLACE] Received new move robot event in message: %s", self.goal_state)


    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        man_group_name = rospy.get_param('manipulator_group_name')
        ee_group_name = rospy.get_param('ee_group_name')

        man_group = moveit_commander.MoveGroupCommander(man_group_name)
        ee_group = moveit_commander.MoveGroupCommander(ee_group_name)

        ee_link = man_group.get_end_effector_link()

        manipulator_joint_names = man_group.get_joints()
        # ee_joint_names = ee_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys() # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()
        EE_GRASP = None
        ee_group.clear_pose_target

        # Allow replanning to increase the odds of a solution
        man_group.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        man_group.set_planning_time(5)

        # Allow some leeway in position (meters) and orientation (radians)
        man_group.set_goal_position_tolerance(0.05)
        man_group.set_goal_orientation_tolerance(0.1)

        ee_group.set_goal_position_tolerance(0.05)
        ee_group.set_goal_orientation_tolerance(0.1)

        self.max_pick_attempts = 10

        self.man_group_name = man_group_name
        self.ee_group_name = ee_group_name
        self.robot = robot

        self.man_group = man_group
        self.ee_group = ee_group

        self.cage_pose = None
        self.ee_link = ee_link
        self.ee_joint_names = ee_joint_names
        self.EE_OPEN = EE_OPEN
        self.EE_CLOSED = EE_CLOSED
        self.EE_GRASP = EE_GRASP

    def initialise_enviroment(self):
        """" Checks wether the RViz enviroment is correctly set


        """

        rospy.logdebug("===INITIALIZING ENVIROMENT====")

        # interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)

        self.target_object_name = 'peduncle'

        known_objects_prev = None
        while True:
            known_objects = self.scene.get_known_object_names()

            if not known_objects == known_objects_prev:
                known_objects_prev = known_objects
                rospy.logdebug("Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("[Pick Place] Table and wall object not present...")
                    break
            rospy.sleep(0.1)

        rospy.logdebug( "Known objects: %s", self.scene.get_known_object_names())


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

        request.ik_request.group_name = self.man_group_name
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)

        if not response.error_code.val == MoveItErrorCodes.SUCCESS:
            return False
        else:
            return True

    def pick(self):
        """ Pick an object
        """

        rospy.logdebug("==STARTING PICK PROCEDURE===")

        # Initialize grasp
        grasps = Grasp()

        # Pre grasp posture
        time_list = [0.1]
        grasps.pre_grasp_posture = self.make_gripper_posture(self.EE_OPEN, time_list)

        # Grasp posture
        time_list = [0.1]
        grasps.grasp_posture = self.make_gripper_posture(self.EE_GRASP, time_list)

        # Set the approach and retreat parameters as desired
        grasps.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1, [0, 0, -1.0])
        grasps.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0, 0, 1.0])

        # grasp pose
        grasps.grasp_pose = self.cage_pose

        # touchable obejcts
        grasps.allowed_touch_objects = ['table', 'wall', 'tomato_0', 'tomato_1', 'peduncle']

        self.man_group.set_support_surface_name("table")

        # Pick
        result = None
        n_attempts = 0

        while result != MoveItErrorCodes.SUCCESS and n_attempts < self.max_pick_attempts:

            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = self.man_group.pick(self.target_object_name, grasps)

            rospy.sleep(0.2)

        self.cage_pose = None

        if result == MoveItErrorCodes.SUCCESS:
            return True
        else:
            return False

    def make_gripper_posture(self, joint_positions, time_list):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = self.ee_joint_names

        for time in time_list:

            rospy.logdebug("Duration: %s", time)

            # Initialize a joint trajectory point to represent the goal
            tp = JointTrajectoryPoint()

            # Assign the trajectory joint positions to the input positions
            tp.positions = joint_positions

            # Set the gripper effort
            tp.effort = self.GRIPPER_EFFORT

            tp.time_from_start = rospy.Duration(time)

            # Append the goal point to the trajectory points
            t.points.append(tp)

        # Return the joint trajectory
        return t

    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = "world"

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    def remove_attached_target_object(self):
        attached_objects = self.scene.get_attached_objects(object_ids=[self.target_object_name])

        if self.target_object_name in attached_objects.keys():
            if attached_objects[self.target_object_name].link_name == self.ee_link:
                rospy.logdebug("Removing atached objects from ee_link")
                self.scene.remove_attached_object(self.ee_link, self.target_object_name)

    def go_to_pose_goal(self):
        """ plan and move to a pose goal


        """
        rospy.logdebug("==STARTING GO TO POSE GOAL PROCEDURE===")

        ## Only if inverse kinematics exist
        if self.compute_ik(self.cage_pose):
            rospy.loginfo("Goal pose is reachable.")

            self.man_group.set_pose_target(self.cage_pose.pose)
            plan = self.man_group.plan()
            self.man_group.execute(plan, wait=True)

        else:
            rospy.logwarn("Goal pose is not reachable!")


        # Ensures that there is no residual movement
        self.man_group.stop()

        # clear targets after planning with poses.
        self.man_group.clear_pose_targets()
        self.cage_pose = None

        # verify goal pose is obtained
        return all_close(self.cage_pose, self.man_group.get_current_pose(), 0.02)

    def open_ee(self):
        self.move_ee("Open")
        self.remove_attached_target_object()
        return True

    def close_ee(self):
        self.move_ee("Closed")
        return True

    def home_man(self):
        return self.move_to_joint_target(self.man_group, 'Upright')

    def move_ee(self, named_target):
        return self.move_to_joint_target(self.ee_group, named_target)

    def move_to_joint_target(self, group, named_target):
        group.set_named_target(named_target)

        plan = group.plan()
        group.execute(plan, wait=True)
        group.stop()

        success = all_close(group.get_joint_value_target(), group.get_current_joint_values(), 0.02)
        group.clear_pose_targets()
        return success

    def command_robot(self):

        # Update state state
        if (self.state != "wait") and self.cage_pose == None and self.EE_GRASP == None:
            self.prev_state = self.state
            self.state = "wait"
            self.take_action()

        if self.state == "wait"  and self.cage_pose != None and self.EE_GRASP != None:
            self.prev_state = self.state
            self.state = "idle"
            self.take_action()

        if (self.state == "idle") and (self.goal_state == "e_pick_place"):
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif (self.state == "idle") and (self.goal_state == "e_move"):
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif ((self.state == "wait") or (self.state == "idle")) and self.goal_state == "e_home":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif ((self.state == "wait") or (self.state == "idle")) and self.goal_state == "e_open":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()

        elif ((self.state == "wait") or (self.state == "idle")) and self.goal_state == "e_close":
            self.prev_state = self.state
            self.state = self.goal_state
            self.take_action()


    def take_action(self):
        msg = String()
        success = None

        # Take action based on state
        if self.state == "wait":
            pass

        if self.state == "idle":
            pass

        elif self.state == "e_pick_place":
            success = self.pick()
            self.state = "wait"

        elif self.state == "e_move":
            success = self.go_to_pose_goal()
            self.state = "wait"

        elif self.state == "e_home":
            success = self.home_man()
            self.state = "idle"

        elif self.state == "e_open":
            success = self.open_ee()
            self.state = "idle"

        elif self.state == "e_close":
            success = self.close_ee()
            self.state = "idle"


        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.goal_state = None
            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Robot command failed")
                self.goal_state = None


            self.pub_e_out.publish(msg)
        else:
            # no robot command has been executed
            pass


def add_list(list1, list2):
    return [sum(x) for x in zip(list1, list2)]

def main():
    try:
        pp = Pick_Place()
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            pp.command_robot()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
