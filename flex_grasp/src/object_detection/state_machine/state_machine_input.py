import rospy
import os


# msg
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log
from cv_bridge import CvBridge, CvBridgeError
from data_logger import DataLogger

DEFAULT_CAMERA_SIM = True

class StateMachineInput(object):

    def __init__(self, node_name):
        self.node_name = node_name

        # data
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None
        self.trans = None

        # state
        self.take_picture = False
        self.command = False
        self.data_path = os.path.join(os.getcwd(), 'thesis_data', 'default')
        self.id = None

        # params
        self.camera_sim = rospy.get_param("camera_sim", DEFAULT_CAMERA_SIM)

        self.bridge = CvBridge()

        self.topics = {'color_image': 'camera/color/image_raw',
                       'depth_image': 'camera/aligned_depth_to_color/image_raw',
                       'camera_info': 'camera/color/camera_info',
                       'pcl': 'camera/depth_registered/points'}

        self.callbacks = {'color_image': self.color_image_cb,
                          'depth_image': self.depth_image_cb,
                          'camera_info': self.camera_info_cb,
                          'pcl': self.point_cloud_cb}

        self.types = {'color_image': Image,
                      'depth_image': Image,
                      'camera_info': CameraInfo,
                      'pcl': PointCloud2}

        self.logger = DataLogger(self.node_name, self.topics, self.types, callbacks=self.callbacks)

        # subscribe
        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("experiment_pwd", String, self.experiment_pwd_cb)
        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)

        for key in self.topics:
            rospy.Subscriber(self.topics[key], self.types[key], self.callbacks[key])

    def e_in_cb(self, msg):
        """If the current command is empty take a command and set appropriate fields"""
        if self.command is None:
            self.command = msg.data
            rospy.logdebug("[%s] Received %s command", self.node_name, self.command)

    def experiment_pwd_cb(self, msg):
        """callback to update the data path"""
        new_path = msg.data
        if self.data_path != new_path:
            self.data_path = new_path

    def color_image_cb(self, msg, force=False):
        if ((self.color_image is None) and self.take_picture) or force:
            rospy.logdebug("[OBJECT DETECTION] Received color image message")
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def depth_image_cb(self, msg, force=False):
        if ((self.depth_image is None) and self.take_picture) or force:
            rospy.logdebug("[OBJECT DETECTION] Received depth image message")
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough") / 1000.0
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def camera_info_cb(self, msg, force=False):
        if (self.camera_info is None) or force:
            rospy.logdebug("[{0}] Received color info message".format(self.node_name))
            self.camera_info = msg

    def point_cloud_cb(self, msg, force=False):
        if ((self.pcl is None) and self.take_picture) or force:
            rospy.logdebug("[{0}] Received point_ cloud info message".format(self.node_name))
            self.pcl = msg

    def received_data(self):
        """Returns a dictionary which contains information about what data has been received"""
        is_received = {}
        is_received['color_img'] = self.color_image is not None
        is_received['depth_img'] = self.depth_image is not None
        is_received['camera_info'] = self.camera_info is not None
        is_received['pcl'] = self.pcl is not None

        is_received['all'] = True
        for key in is_received:
            is_received['all'] = is_received['all'] and is_received[key]

        return is_received

    def print_received_data(self, is_received):
        """Prints a warning for the data which has not been received"""
        for key in is_received:
            if not is_received[key]:
                rospy.logwarn("[{0}] Did not receive {1} yet.".format(self.node_name, key))

    def wait_for_data(self, timeout=1):
        start_time = rospy.get_time()
        is_received = {}

        while rospy.get_time() - start_time < timeout:
            is_received = self.received_data()
            if is_received['all']:
                self.take_picture = False
                rospy.logdebug("[{0}] Received all data".format(self.node_name))
                return True

            rospy.sleep(0.1)

        self.print_received_data(is_received)
        return False

    def get_messages(self):
        return {'color_image': self.bridge.cv2_to_imgmsg(self.color_image, encoding="rgb8"),
                'depth_image': self.bridge.cv2_to_imgmsg(self.depth_image, encoding="passthrough"),
                'camera_info': self.camera_info,
                'pcl': self.pcl}

    def log_data(self):
        """Store received data in a rosbag"""
        is_received = self.received_data()
        self.id = self.get_new_attempt_id()

        if is_received['all']:
            messages = self.get_messages()
            self.logger.write_messages(messages, self.data_path, self.id)
        else:
            rospy.logwarn("[{0}] Cannot log data: not all data is received".format(self.node_name))
            self.print_received_data(is_received)

    def load_data(self):
        self.id = '001'
        self.logger.publish_messages(self.data_path, self.id)

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

    def get_new_attempt_id(self):
        """returns the lowest available id in the given path"""

        if not os.path.isdir(self.data_path):
            new_id_int = 1
        else:
            contents = os.listdir(self.data_path)
            if len(contents) == 0:
                new_id_int = 1
            else:
                contents.sort()
                file_name = contents[-1]
                file_id = file_name[:3]
                new_id_int = int(file_id) + 1

        return str(new_id_int).zfill(3)
