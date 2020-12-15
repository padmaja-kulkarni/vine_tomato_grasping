import rospy
import os


# msg
from flex_grasp.msg import FlexGraspErrorCodes
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

from flex_shared_resources.errors.flex_grasp_error import flex_grasp_error_log, flex_grasp_error_string
from cv_bridge import CvBridge, CvBridgeError
from data_logger import DataLogger

DEFAULT_CAMERA_SIM = False

class StateMachineInput(object):

    def __init__(self, node_name):
        self.node_name = node_name

        # data
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None

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

        self.logger = DataLogger(self.node_name, self.topics, self.types)

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
            rospy.logdebug("[{0}] Received color image message".format(self.node_name))
            try:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def depth_image_cb(self, msg, force=False):
        if ((self.depth_image is None) and self.take_picture) or force:
            rospy.logdebug("[{0}] Received depth image message".format(self.node_name))
            try:
                if self.camera_sim:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                else:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough") / 1000.0
            except CvBridgeError as e:
                rospy.logwarn("[{0}] {1}".format(self.node_name, e))

    def camera_info_cb(self, msg, force=False):
        if (self.camera_info is None) or force:
            rospy.logdebug("[{0}] Received camera info message".format(self.node_name))
            self.camera_info = msg

    def point_cloud_cb(self, msg, force=False):
        if ((self.pcl is None) and self.take_picture) or force:
            rospy.logdebug("[{0}] Received point cloud info message".format(self.node_name))
            self.pcl = msg

    def received_messages(self):
        """Returns a dictionary which contains information about what data has been received"""
        is_received = {'color_img': self.color_image is not None,
                       'depth_img': self.depth_image is not None,
                       'camera_info': self.camera_info is not None,
                       'pcl': self.pcl is not None,
                       'all': True}

        for key in is_received:
            is_received['all'] = is_received['all'] and is_received[key]

        return is_received

    def print_received_messages(self, is_received):
        """Prints a warning for the data which has not been received"""
        for key in is_received:
            if not is_received[key]:
                rospy.logwarn("[{0}] Did not receive {1} yet.".format(self.node_name, key))

    def wait_for_messages(self, timeout=1):
        start_time = rospy.get_time()
        is_received = {}

        while rospy.get_time() - start_time < timeout:
            is_received = self.received_messages()
            if is_received['all']:
                self.take_picture = False
                rospy.logdebug("[{0}] Received all data".format(self.node_name))
                return True

            rospy.sleep(0.1)

        self.print_received_messages(is_received)
        return False

    def get_messages(self):
        messages = {}
        if self.color_image is not None:
            messages['color_image'] = self.bridge.cv2_to_imgmsg(self.color_image, encoding="rgb8")
        else:
            messages['color_image'] = None
        if self.depth_image is not None:
            messages['depth_image'] = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="passthrough")
        else:
            messages['depth_image'] = None

        messages['camera_info'] = self.camera_info
        messages['pcl'] = self.pcl
        return messages

    def log_messages(self):
        """"log messages"""
        self.id = self.get_new_attempt_id()
        self.logger.write_messages(self.get_messages(), self.data_path, self.id)

    def load_messages(self):
        """load messages"""
        self.id = self.get_next_attempt_id()
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
        result_string = flex_grasp_error_string(result)
        rospy.logdebug("[{0}] Completed command {1} with {2}".format(self.node_name, self.command, result_string))
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

    def get_next_attempt_id(self):
        """increment the id by one if it exsists"""
        if self.id is None:
            id_int = 1
        else:
            id_int = int(self.id) + 1
        return str(id_int).zfill(3)
