import rospy
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

CAMERA_SIM = True

class StateMachineInput(object):

    def __init__(self, node_name):
        self.node_name = node_name

        # data
        self.color_image = None
        self.depth_image = None
        self.color_info = None
        self.pcl = None
        self.trans = None

        # state
        self.take_picture = False
        self.command = False

        # params
        self.camera_sim = rospy.get_param("camera_sim", CAMERA_SIM)
        self.bridge = CvBridge()

        # subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("camera/color/image_raw", Image, self.color_image_cb)
        rospy.Subscriber("camera/aligned_depth_to_color/image_raw", Image, self.depth_image_cb)
        rospy.Subscriber("camera/color/camera_info", CameraInfo, self.color_info_cb)
        rospy.Subscriber("camera/depth_registered/points", PointCloud2, self.point_cloud_cb)

        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[%s] Received %s command", self.node_name, self.command)

    def color_image_cb(self, msg):
        if (self.color_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received color image message")
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def depth_image_cb(self, msg):
        if (self.depth_image is None) and (self.take_picture):
            rospy.logdebug("[OBJECT DETECTION] Received depth image message")
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough") / 1000.0
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def color_info_cb(self, msg):
        if self.color_info is None:
            rospy.logdebug("[{0}] Received color info message".format(self.node_name))
            self.color_info = msg

    def point_cloud_cb(self, msg):
        if (self.pcl is None) and (self.take_picture):
            rospy.logdebug("[{0}] Received point_ cloud info message".format(self.node_name))
            self.pcl = msg

    def received_data(self):
        """Returns a dictionary which contains information about what data has been received"""

        is_received = {}
        is_received['color_img'] = self.color_image is not None
        is_received['depth_img'] = self.depth_image is not None
        is_received['color_info'] = self.color_info is not None
        is_received['pcl'] = self.pcl is not None

        all_data = True
        for key in is_received:
            all_data = all_data and is_received[key]

        is_received['all'] = all_data
        return is_received

    def log_received_data(self, is_received):
        for key in is_received:
            if not is_received[key]:
                rospy.logwarn("[{0}] Did not receive {1} yet.".format(self.node_name, key))

    def wait_for_data(self, timeout=1):
        start_time = rospy.get_time()

        while rospy.get_time() - start_time < timeout:
            is_received = self.received_data()
            if is_received['all']:
                self.take_picture = False
                rospy.logdebug("[{0}] Received all data".format(self.node_name))
                return True

            rospy.sleep(0.1)

        self.log_received_data(self, is_received)
        return False

    def reset(self):
        """Resets the input state to its original state on init."""
        self.command = None
        self.color_image = None
        self.depth_image = None
        self.pcl = None

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
