import rospy
import rosbag
import os

from flex_grasp.msg import FlexGraspErrorCodes

class DataLogger(object):
    """Generic data logger class"""

    def __init__(self, node_name, topics, types, callbacks=None):
        self.node_name = node_name
        self.topics = topics
        self.callbacks = callbacks
        self.types = types
        self.bag = None

        self.publisher = {}
        for key in self.topics:
            self.publisher[key] = rospy.Publisher(self.topics[key], self.types[key], queue_size=1, latch=True)

    def write_messages(self, messages):
        """Write data in a rosbag"""
        # try:
        for key in self.topics:
            self.write_message(key, messages[key])
        # finally:
        #     self.bag.close()

    def publish_messages_from_bag(self):
        """Read data from a rosbag and publish the received data"""
        for key in self.topics:
            topic = self.topics[key]
            rospy.logdebug("[{0}] reading {1} from bag on topic {2}".format(self.node_name, key, topic))
            for topic, message, t in self.bag.read_messages(topics=topic):
                self.publisher[key].publish(message)

    def write_message(self, key, message):
        """Write received data in a rosbag"""
        # try:
        self.bag.write(self.topics[key], message)
        # finally:
        # self.bag.close()

    def publish_messages(self, messages):
        success = FlexGraspErrorCodes.SUCCESS
        for key in messages:
            result = self.publish_message(key, messages[key])
            if result == FlexGraspErrorCodes.FAILURE:
                success = FlexGraspErrorCodes.FAILURE

        return success

    def publish_message(self, key, message):
        if isinstance(message, self.types[key]):
            rospy.loginfo("[{0}] Publishing {1}".format(self.node_name, key))
            self.write_message(key, message)
            self.publisher[key].publish(message)
            return FlexGraspErrorCodes.SUCCESS
        else:
            rospy.logwarn("[{0}] Cannot publish method: no instance of specified type".format(self.node_name))
            return FlexGraspErrorCodes.FAILURE


    def open_bag(self, bag_path, bag_id, bag_name=None):

        if self.bag is not None:
            rospy.loginfo("[{0}] Closing previous bag".format(self.node_name))
            self.bag.close()

        if bag_name is None:
            bag_name = self.node_name
        full_name = bag_id + '_' + bag_name + '.bag'
        full_path = os.path.join(bag_path, full_name)

        rospy.loginfo("[{0}] Opening bag {1}".format(self.node_name, full_path))

        if not os.path.isdir(bag_path):
            rospy.loginfo("[{0}] New path, creating a new folder {1}".format(self.node_name, bag_path))
            os.makedirs(bag_path)

        self.bag = rosbag.Bag(full_path, 'w')

    # def receive_messages(self):
    #     """Read data from a rosbag and trigger the callbacks the received data"""
    #     if self.callbacks is None:
    #         rospy.logwarn("[{0}] Data logger can not trigger callbacks: they are not defined!".format(self.node_name))
    #         return
    #
    #     rospy.logdebug("[{0}] Reading and publishing messages from file {1}".format(self.node_name, self.bag_name))
    #     bag = rosbag.Bag(self.bag_name)
    #
    #     for key in self.topics:
    #         topic = self.topics[key]
    #         rospy.logdebug("[{0}] reading {1} from bag on topic {2}".format(self.node_name, key, topic))
    #         for topic, message, t in bag.read_messages(topics=topic):
    #             callback = self.callbacks[key]
    #             callback(message, force=True)
    #
    #     bag.close()