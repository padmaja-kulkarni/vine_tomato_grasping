#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 13:07:08 2020

@author: jelle
"""

# packages
import sys
import rospy
import moveit_commander

# functions
from moveit_commander.conversions import pose_to_list

# services
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# messages
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes


# custom functions
from func.utils import pose_close, joint_close, deg2rad
from func.ros_utils import wait_for_param

class MoveRobot(object):
    """MoveRobot"""
    def __init__(self):
        super(MoveRobot, self).__init__()

        self.debug_mode = rospy.get_param("move_robot/debug")

        if self.debug_mode:
            log_level = rospy.DEBUG
            rospy.loginfo("[MOVE ROBOT] Luanching move robot in debug mode")
        else:
            log_level = rospy.INFO

        rospy.init_node("move_robot",
                        anonymous=True,
                        log_level=log_level)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass

        self.state = "idle"
        self.prev_state = None
        self.command = None

        self.robot_pose = None

        # tolerance
        self.position_tol = 0.05 # [m]
        self.orientation_tol = deg2rad(1.0) # [rad]
        self.man_joint_tolerance = deg2rad(5.0) # [rad]
        self.ee_joint_tolerance = 0.001 # [m]

        self.initialise_robot()
        self.initialise_enviroment()

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        # Subscribers
        rospy.Subscriber("robot_pose", PoseStamped, self.robot_pose_cb)
        
        # rospy.Subscriber("endEffectorDistance", Float64, self.ee_distance_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers
        latch = True
        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=latch)
            
    def robot_pose_cb(self, msg):
        if self.robot_pose is None:
            self.robot_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new robot pose massage")


    def e_in_cb(self, msg):
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[MOVE ROBOT] Received new move robot event in message: %s", self.command)

            # reset outputting message
            msg = String()
            msg.data = ""
            self.pub_e_out.publish(msg)
            

    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        ## This object is an interface to one group of joints.
        man_group_name = rospy.get_param('manipulator_group_name')
        ee_group_name = rospy.get_param('ee_group_name')

        rospy.logdebug("[MOVE ROBOT] Manipulator group name: %s", man_group_name)
        rospy.logdebug("[MOVE ROBOT] End effector group name: %s", ee_group_name)

        man_group = moveit_commander.MoveGroupCommander(man_group_name)
        ee_group = moveit_commander.MoveGroupCommander(ee_group_name)

        man_planning_frame = man_group.get_planning_frame()
        ee_planning_frame = ee_group.get_planning_frame()

        rospy.logdebug("[MOVE ROBOT] Manipulator planning frame: %s", man_planning_frame)
        rospy.logdebug("[MOVE ROBOT] End effector planning frame: %s", ee_planning_frame)

        ee_link = man_group.get_end_effector_link()

        manipulator_joint_names = man_group.get_joints()
        ee_joint_names = ee_group.get_named_target_values("Closed").keys() # the ee_group contains joints we cannot actually control?

        EE_CLOSED = ee_group.get_named_target_values("Closed").values()
        EE_OPEN = ee_group.get_named_target_values("Open").values()

        # Allow replanning to increase the odds of a solution
        man_group.allow_replanning(True)

        # Allow 5 seconds per planning attempt
        man_group.set_planning_time(5)

        # Allow some leeway in position (meters) and orientation (radians) PLANNING!
#        man_group.set_goal_position_tolerance(self.position_tol)
#        man_group.set_goal_orientation_tolerance(self.orientation_tol)
#        man_group.set_goal_joint_tolerance(self.man_joint_tolerance)
#        
#        ee_group.set_goal_position_tolerance(self.position_tol)
#        ee_group.set_goal_orientation_tolerance(self.orientation_tol)
#        ee_group.set_goal_joint_tolerance(self.ee_joint_tol)

        self.max_attempts = 2

        self.man_group_name = man_group_name
        self.ee_group_name = ee_group_name
        self.robot = robot

        self.man_group = man_group
        self.ee_group = ee_group

        self.man_planning_frame = man_planning_frame
        self.ee_planning_frame = ee_planning_frame

        self.ee_link = ee_link
        self.ee_joint_names = ee_joint_names
        self.EE_OPEN = EE_OPEN
        self.EE_CLOSED = EE_CLOSED

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
                rospy.logdebug("[MOVE ROBOT] Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("[MOVE ROBOT] Table and wall object not present...")
                    break
            rospy.sleep(0.1)

        rospy.logdebug("[MOVE ROBOT] Known objects: %s", self.scene.get_known_object_names())


    def check_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Checks inverse kinematics for the given pose.

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
            rospy.logwarn("[MOVE ROBOT] Goal pose is not reachable: inverse kinematics can not be found")
            return False
        else:
            return True

    def check_frames(self, goal_frame):
        # remove initial "/"
        planning_frame = self.man_planning_frame[1:]

        if goal_frame == planning_frame:
            return True
        else:
            rospy.logwarn("[MOVE ROBOT] Goal pose specified with respect to wrong frame: sould be specified with respect to %s, but is be specified with respect to %s", planning_frame, goal_frame)
            return False

    def remove_attached_target_object(self):
        attached_objects = self.scene.get_attached_objects(object_ids=[self.target_object_name])

        if self.target_object_name in attached_objects.keys():
            if attached_objects[self.target_object_name].link_name == self.ee_link:
                rospy.logdebug("[MOVE ROBOT] Removing atached objects from ee_link")
                self.scene.remove_attached_object(self.ee_link, self.target_object_name)
                rospy.sleep(0.1)
                self.scene.remove_world_object(self.target_object_name)
                return True
            else:
                rospy.logdebug("[MOVE ROBOT] Cannot remove attached object: target object is not attached to the end effector link!")
                return True
        else:
            rospy.logdebug("[MOVE ROBOT] Cannot remove attached object: target object is not attached to anything!")
            return True

    def attach_object(self):
        grasping_group = self.ee_group_name
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.ee_link, self.target_object_name, touch_links=touch_links)
        rospy.sleep(0.1)
        return True

    def go_to_random_pose(self):
        rospy.logdebug("[MOVE ROBOT] Going to random pose")
        goal_pose = self.man_group.get_random_pose()
        success = self.go_to_pose(goal_pose)
        # self.grasp_pose = None
        return success

    def sleep_man(self):
        rospy.logdebug("[MOVE ROBOT] Sleeping manipulator")
        return self.move_to_joint_target(self.man_group, 'Sleep')

    def home_man(self):
        rospy.logdebug("[MOVE ROBOT] Homeing manipulator")
        return self.move_to_joint_target(self.man_group, 'Upright')

    def open_ee(self):
        rospy.logdebug("[MOVE ROBOT] Opening end effector")
        return self.move_to_joint_target(self.ee_group, "Open")

    def close_ee(self):
        rospy.logdebug("[MOVE ROBOT] Closing end effector")
        return self.move_to_joint_target(self.ee_group, "Closed")

    def apply_release_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying release with end effector")

        success = self.open_ee()
        if success:
            success = self.remove_attached_target_object()
        return success

    def apply_grasp_ee(self):
        rospy.logdebug("[MOVE ROBOT] Aplying grasping with end effector")

        success = self.attach_object()
        if success:
            success = self.close_ee() # move_to_joint_target(self.ee_group, self.grasp_ee)
        return success


    def go_to_pose(self, goal_pose):
        rospy.logdebug("[MOVE ROBOT] Go to pose")        

        if goal_pose is None:
            rospy.logwarn("goal pose is empty!")
            return False
        if not self.check_frames(goal_pose.header.frame_id):
            return False
        if not self.check_ik(goal_pose):
            return False

        self.man_group.set_pose_target(goal_pose.pose)
        
        success = False
        attempt = 0
        while success is False:
            attempt = attempt + 1
            

            plan = self.man_group.plan()
            
            # if not plan found... very ugly, what is a better way to do this?
            if len(plan.joint_trajectory.joint_names) == 0:
                rospy.logwarn('[MOVE ROBOT] not attempting to go to pose goal, no plan found!')
                return False
            
            self.man_group.execute(plan, wait=True)
    
            # Ensures that there is no residual movement and clear the target
            self.man_group.stop()
            self.man_group.clear_pose_targets()
            rospy.sleep(1)            
            
            curr_pose = self.man_group.get_current_pose()
            
            orientation_close, position_close = pose_close(goal_pose, curr_pose, self.position_tol, self.orientation_tol)
            is_all_close = position_close # orientation_close and position_close
            success = is_all_close
            if is_all_close is False:
                if orientation_close is False:
                    # self.man_group.get_goal_orientation_tolerance()
                    rospy.logdebug("[MOVE ROBOT] Failed to move to pose target, obtained orientation is not sufficiently close to goal orientation (tolerance: %s). Attempts remaining: %s",self.orientation_tol, self.max_attempts - attempt) 
                    
                if position_close is False:
                    # self.man_group.get_goal_position_tolerance()
                    rospy.loginfo("[MOVE ROBOT] Failed to move to pose target, obtained position is not sufficiently close to goal position (tolerance: %s). Attempts remaining: %s", self.position_tol, self.max_attempts - attempt)
                    
                rospy.logdebug("[MOVE ROBOT] Goal pose: %s", pose_to_list(goal_pose.pose))
                rospy.logdebug("[MOVE ROBOT] Curr pose: %s", pose_to_list(curr_pose.pose))
                
            if attempt >= self.max_attempts:
                return False

        return success

    def move_to_joint_target(self, group, target):
        rospy.logdebug("[MOVE ROBOT] Go to joint target") 
        to_check = True

        # if the target is a named target, get the corresponding joint values
        if type(target) is str:
            if target == "Sleep":
                to_check = False
            target = group.get_named_target_values(target)

        group.set_joint_value_target(target)

        plan = group.plan()
        success = group.execute(plan, wait=True)
        group.stop()
    
        current = group.get_current_joint_values()
        
        #  for some reason group.get_joint_value_target() returns zeros on ee_group_name
        if (group.get_name() == self.ee_group_name):
            success = True
            target = target.values()
        else:
            target = group.get_joint_value_target()
        
        if (group.get_name() == self.ee_group_name):
            tol = self.ee_joint_tolerance
        else:
            tol = self.man_joint_tolerance
        
        if to_check:
            is_all_close = joint_close(target, current, tol)
            if is_all_close is False:
                rospy.logwarn("[MOVE ROBOT] Failed to move to joint target: obtained joint values are not sufficiently close to target joint value (tolerance: %s). Attempts remaining %s", tol, 0)
                rospy.loginfo("[MOVE ROBOT] Target joint values: %s", target)
                rospy.loginfo("[MOVE ROBOT] Actual joint values: %s", current)
                success = False

        group.clear_pose_targets()
        return success

    def reset(self):
        rospy.logdebug("[MOVE ROBOT] Resetting robot pose") 
        self.robot_pose = None
        return True

    def take_action(self):
        msg = String()
        success = None

        # General actions, non state dependent
        if self.command == "move_manipulator":
            success = self.go_to_pose(self.robot_pose)
            self.robot_pose = None

        elif self.command == "sleep":
            success = self.sleep_man()

        elif self.command == "home":
            success = self.home_man()
            
        elif self.command == "grasp":
            success = self.apply_grasp_ee()

        elif self.command == "release":
            success = self.apply_release_ee()

        elif self.command == "open":
            success = self.apply_release_ee()

        elif self.command == "close":
            success = self.close_ee()
            
        elif self.command == "reset":
            success = self.reset()

        elif self.command == "e_init":
            success = True
            
        elif self.command == None:
            pass
            
        else:
            rospy.logwarn("Received unknwon command: %s", self.command)
            success = False

        # publish success
        if success is not None:
            if success == True:
                msg.data = "e_success"
                self.command = None

            elif success == False:
                msg.data = "e_failure"
                rospy.logwarn("Robot command failed")
                self.command = None

            self.pub_e_out.publish(msg)


def main():
    try:
        move_robot = MoveRobot()

        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            move_robot.take_action()
            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
