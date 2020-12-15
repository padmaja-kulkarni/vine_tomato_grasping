import rospy
import rosbag
import os


class DataLogger(object):
    """Generic data logger class"""

    def __init__(self, node_name, topics, types, bag_name=None, callbacks=None):
        self.node_name = node_name
        self.topics = topics
        self.callbacks = callbacks
        self.types = types
        if bag_name is None:
            bag_name = node_name
        self.bag_name = bag_name + '.bag'

        self.publisher = {}
        for key in self.topics:
            self.publisher[key] = rospy.Publisher(self.topics[key], self.types[key], queue_size=1, latch=True)

    def write_messages(self, messages, path, id):
        """Write received data in a rosbag"""
        full_name = id + '_' + self.bag_name
        full_path = os.path.join(path, full_name)

        rospy.loginfo("[{0}] Writing received messages to file {1}".format(self.node_name, full_path))
        if not os.path.isdir(path):
            rospy.loginfo("[{0}] New path, creating a new folder {1}".format(self.node_name, path))
            os.makedirs(path)

        bag = rosbag.Bag(full_path, 'w')

        try:
            for key in self.topics:
                bag.write(self.topics[key], messages[key])
        finally:
            bag.close()

    def publish_messages(self, bag_path, bag_id):
        """Read data from a rosbag and publish the received data"""

        full_name = bag_id + '_' + self.bag_name
        full_path = os.path.join(bag_path, full_name)
        rospy.loginfo("[{0}] Reading and publishing messages from file {1}".format(self.node_name, full_path))

        if os.path.exists(full_path):
            bag = rosbag.Bag(full_path)
        else:
            rospy.logwarn("[{0}] Cannot publish bag: the file {1} does not exist!".format(self.node_name, full_path))
            return False

        for key in self.topics:
            topic = self.topics[key]
            rospy.logdebug("[{0}] reading {1} from bag on topic {2}".format(self.node_name, key, topic))
            for topic, message, t in bag.read_messages(topics=topic):
                self.publisher[key].publish(message)

        bag.close()

    def receive_messages(self):
        """Read data from a rosbag and trigger the callbacks the received data"""
        if self.callbacks is None:
            rospy.logwarn("[{0}] Data logger can not trigger callbacks: they are not defined!".format(self.node_name))
            return

        rospy.logdebug("[{0}] Reading and publishing messages from file {1}".format(self.node_name, self.bag_name))
        bag = rosbag.Bag(self.bag_name)

        for key in self.topics:
            topic = self.topics[key]
            rospy.logdebug("[{0}] reading {1} from bag on topic {2}".format(self.node_name, key, topic))
            for topic, message, t in bag.read_messages(topics=topic):
                callback = self.callbacks[key]
                callback(message, force=True)

        bag.close()


