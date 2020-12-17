import rospy

# msg
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log, flex_grasp_error_string

class StateMachineInput(object):

    def __init__(self, node_name):
        self.node_name = node_name

        # data
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None

        # state
        self.command = False

        # subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[%s] Received %s command", self.node_name, self.command)

    def reset(self):
        """Resets the input state to its original state on init."""
        self.command = None

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
        result_string = flex_grasp_error_string(result)
        rospy.logdebug("[{0}] Completed command {1} with {2}".format(self.node_name, self.command, result_string))
        flex_grasp_error_log(result, self.node_name)
        msg = FlexGraspErrorCodes(result)
        self.pub_e_out.publish(msg)
        self.reset()

