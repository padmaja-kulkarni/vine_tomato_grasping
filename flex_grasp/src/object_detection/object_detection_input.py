import rospy
import os

# msg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped

from state_machine.data_logger import DataLogger
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_CAMERA_SIM = False

class ObjectDetectionInput(object):
    """Data input output interface for object dectection"""

    def __init__(self, node_name):
        self.node_name = node_name

        # state
        self.take_picture = False
        self.dream = False

        # params
        self.camera_sim = rospy.get_param("camera_sim", DEFAULT_CAMERA_SIM)

        # data_in
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None

        self.bridge = CvBridge()

        self.bag_path = os.path.join(os.getcwd(), 'thesis_data', 'default')
        self.bag_id = None
        self.bag_name = None

        # inputs
        topics = {'color_image': 'camera/color/image_raw',
                       'depth_image': 'camera/aligned_depth_to_color/image_raw',
                       'camera_info': 'camera/color/camera_info',
                       'pcl': 'camera/depth_registered/points'}

        types = {'color_image': Image,
                      'depth_image': Image,
                      'camera_info': CameraInfo,
                      'pcl': PointCloud2}

        callbacks = {'color_image': self.color_image_cb,
                          'depth_image': self.depth_image_cb,
                          'camera_info': self.camera_info_cb,
                          'pcl': self.point_cloud_cb}

        for key in types:
            rospy.Subscriber(topics[key], types[key], callbacks[key])

        rospy.Subscriber("experiment_pwd", String, self.experiment_pwd_cb)
        self.logger = DataLogger(self.node_name, topics, types, bag_name='camera')

    def experiment_pwd_cb(self, msg):
        """callback to update the data path"""
        new_path = msg.data
        if self.bag_path != new_path:
            self.bag_path = new_path
            rospy.loginfo("[{0}] Storing results in folder {1}".format(self.node_name, new_path))

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

    def reset(self):
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.pcl = None


    def collect_messages(self):
        if self.dream:
            self.bag_id = self.get_next_bag_id()
            self.input_logger.publish_messages_from_bag(self.bag_path, self.bag_id)
        else:
            self.take_picture = True

    def log_messages(self):
        """"log messages"""
        rospy.logdebug("[{0}] Logging input messages".format(self.node_name))
        self.bag_id = self.get_new_bag_id()
        self.input_logger.write_messages_to_bag(self.get_messages(), self.bag_path, self.bag_id)

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

    def get_next_bag_id(self):
        """increment the id by one if it exsists"""
        if self.bag_id is None:
            id_int = 1
        else:
            id_int = int(self.bag_id) + 1
        return str(id_int).zfill(3)

    def get_new_bag_id(self):
        """returns the lowest available id in the given path"""

        if not os.path.isdir(self.bag_path):
            new_id_int = 1
        else:
            contents = os.listdir(self.bag_path)
            if len(contents) == 0:
                new_id_int = 1
            else:
                contents.sort()
                file_name = contents[-1]
                file_id = file_name[:3]
                new_id_int = int(file_id) + 1

        return str(new_id_int).zfill(3)