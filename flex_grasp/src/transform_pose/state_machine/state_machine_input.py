import rospy
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log

class StateMachineInput(object):

    def __init__(self, node_name):
        self.node_name = node_name

        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("truss_pose", PoseStamped, self.truss_pose_cb)

        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

        # state
        self.command = None
        self.object_pose = None

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            if msg.data == "transform" or msg.data == "e_init":
                self.command = msg.data
                rospy.logdebug("[%s] Received %s command", self.node_name, self.command)
            else:
                rospy.logwarn("[{0}] Received unknown command {1}!".format(self.node_name, msg.data))
                self.command_rejected(FlexGraspErrorCodes.UNKNOWN_COMMAND)

    def truss_pose_cb(self, msg):
        self.object_pose = msg
        rospy.logdebug("[%s] Received new object feature message", self.node_name)

    def reset(self):
        """Resets the input state to its original state on init."""
        self.command = None
        self.object_pose = None

    def command_rejected(self, error_code=FlexGraspErrorCodes.FAILURE):
        """method called when the state machine rejects the requested model."""
        rospy.logdebug("[%s] Rejected command in message", self.node_name)
        msg = FlexGraspErrorCodes(error_code)
        self.pub_e_out.publish(msg)
        self.reset()

    def command_accepted(self):
        """ method called when state machine accepts the requested command """
        rospy.logdebug("[%s] Accepted command in message: %s", self.node_name, self.command)
        msg = FlexGraspErrorCodes(FlexGraspErrorCodes.NONE)
        self.pub_e_out.publish(msg)

    def command_completed(self, result=FlexGraspErrorCodes.SUCCESS):
        """method called when the state machine completes the requested model."""
        rospy.logdebug("[{0}] Completed command {1} with.".format(self.node_name, self.command))
        flex_grasp_error_log(result, self.node_name)
        msg = FlexGraspErrorCodes(result)
        self.pub_e_out.publish(msg)
        self.reset()
