#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from moveit_msgs.msg import MoveItErrorCodes

# custom functions
from func.all_close import all_close

# enum
# from RobotState import RobotState


class MoveRobot(object):
    """MoveRobot"""
    def __init__(self):
        super(MoveRobot, self).__init__()

        rospy.init_node("Move_Robot",
                        anonymous=True,
                        log_level=rospy.DEBUG)

        # wait until clock is initialized
        while rospy.get_time() < 0.1:
            pass


        self.initialise_enviroment()
        self.initialise_robot()

        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.event = None

        # Subscribers
        rospy.Subscriber("endEffectorPose", PoseStamped, self.pose_cb)
        rospy.Subscriber("~e_in", String, self.e_in_cb)

        # Publishers

        self.pub_e_out = rospy.Publisher("~e_out",
                                   String, queue_size=10, latch=True)


    def pose_cb(self, msg):
        if self.robot_goal_pose is None:
            self.robot_goal_pose = msg
            rospy.logdebug("[MOVE ROBOT] Received new move robot pose message")

    def e_in_cb(self, msg):
        if self.event is None:
            self.event = msg.data
            rospy.logdebug("[MOVE ROBOT] Received new move robot event in message: %s", self.event)


    def initialise_robot(self):
        rospy.logdebug("===INITIALIZING ROBOT====")

        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        group_name = rospy.get_param('move_group_name')
        group = moveit_commander.MoveGroupCommander(group_name)

        # rospy.sleep(10)required?

        self.robot = robot
        self.group = group
        self.robot_goal_pose = None
        # self.state = RobotState.INITIALIZING

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

        if not response.error_code.val == MoveItErrorCodes.SUCCESS:
            return False
        # We do not need the actual solution
        # joint_state = response.solution.joint_state
        # for name, position in zip(joint_state.name, joint_state.position):
        #     if name in ArmJoints.names():
        #         rospy.loginfo('{}: {}'.format(name, position))
        return True


    def initialise_enviroment(self):
        """" Checks wether the RViz enviroment is correctly set


        """

        rospy.logdebug("===INITIALIZING ENVIROMENT====")

        # interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(5)

        # Check wether table and wall objects are present
        known_objects_prev = None
        while True:
            known_objects = self.scene.get_known_object_names()

            if not known_objects == known_objects_prev:
                known_objects_prev = known_objects
                rospy.logdebug("Known objects: %s", known_objects)

                if ('table' in known_objects) and ('wall' in known_objects):
                    break
                else:
                    rospy.logwarn("Table and wall object not present, refusing to continue before they are added")
            rospy.sleep(0.1)


#        if self.add_box(box_name = 'table'):
#            rospy.logdebug("Succesfully added table")
#        else:
#            rospy.logwarn("Unable to add table")
#
#        rospy.logdebug( "Known objects: %s", self.scene.get_known_object_names())

    def go_to_pose_goal(self):
        """ plan and move to a pose goal


        """

        ## Only if inverse kinematics exist
        if self.compute_ik(self.robot_goal_pose):
            rospy.loginfo("Goal pose is reachable.")

            self.group.set_pose_target(self.robot_goal_pose.pose)
            plan = self.group.plan()
            self.group.execute(plan, wait=True)

        else:
            rospy.logwarn("Goal pose is not reachable!")


        # Ensures that there is no residual movement
        self.group.stop()

        # clear targets after planning with poses.
        self.group.clear_pose_targets()
        self.robot_goal_pose = None

        # verify goal pose is obtained
        return all_close(self.robot_goal_pose, self.group.get_current_pose(), 0.02)

    def go_to_home(self):
        """ Plan and move to home

        """
        self.group.set_named_target('Upright')

        ## Planning to a joint Goal
#        group_variable_values = self.group.get_current_joint_values()
#        for i in range(0, len(group_variable_values)):
#            group_variable_values[i] = 0
#
#        # in this case ik always allow, no checking is required
#        self.group.set_joint_value_target(group_variable_values)
        plan = self.group.plan()
        self.group.execute(plan, wait=True)

        # Ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()

        return all_close(self.group.get_joint_value_target(), self.group.get_current_joint_values(), 0.02)


    def command_robot(self):


        if (self.robot_goal_pose is not None and self.event == "e_start") or self.event == "e_home":
            msg = String()
            success = None

            if self.event == "e_start":
                succes = self.go_to_pose_goal()
            elif self.event == "e_home":
                succes = self.go_to_home()

            if succes:
                msg.data = "e_success"
            else:
                msg.data = "e_failure"
                rospy.logwarn("Goal Tolerance Violated")

            self.event = None
            self.pub_e_out.publish(msg)

        else:
            pass


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
