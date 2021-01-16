import rospy
import rosbag
import os

from flex_grasp.msg import FlexGraspErrorCodes

class DataLogger(object):
    """Generic data logger class"""

    def __init__(self, node_name, topics, types, bag_name=None, callbacks=None, queue_size=1):
        """topics and types should be provided as
        1) String and Type dictionaries with corresponding keys
        2) String and Type
        """
        if isinstance(topics, str):
            my_topic = topics
            my_type = types
            topics = {my_topic: my_topic}
            types = {my_topic: my_type}

        self.node_name = node_name
        self.topics = topics
        self.callbacks = callbacks
        self.types = types
        self.bag = None

        if bag_name is None:
            bag_name = self.node_name
        self.bag_name = bag_name

        if types is not None:
            self.publisher = {}
            for key in self.topics:
                self.publisher[key] = rospy.Publisher(self.topics[key], self.types[key], queue_size=queue_size, latch=True)

    def write_messages_to_bag(self, messages, bag_path, bag_id):
        """Write data in a rosbag"""
        success = self._open_bag(bag_path, bag_id, write=True)
        if success:
            try:
                for key in self.topics:
                    self._write_message(key, messages[key])
            finally:
                self._close_bag()
            return FlexGraspErrorCodes.SUCCESS
        else:
            return FlexGraspErrorCodes.FAILURE

    def load_messages_from_bag(self, bag_path, bag_id):
        """Read data from a rosbag and publish the received data"""
        success = self._open_bag(bag_path, bag_id, read=True)
        if success:
            for key in self.topics:
                topic = self.topics[key]
                rospy.logdebug("[{0}] Reading {1} from bag on topic {2}".format(self.node_name, key, topic))
                messages = {}
                for topic, message, t in self.bag.read_messages(topics=topic):
                    messages[key] = message
            self._close_bag()
            return messages
        else:
            return None

    def publish_messages_from_bag(self, bag_path, bag_id):
        """Read data from a rosbag and publish the received data"""
        success = self._open_bag(bag_path, bag_id, read=True)
        if success:
            for key in self.topics:
                topic = self.topics[key]
                rospy.logdebug("[{0}] Reading {1} from bag on topic {2}".format(self.node_name, key, topic))
                for topic, message, t in self.bag.read_messages(topics=topic):
                    rospy.logdebug("[{0}] Publishing {1} from bag on topic {2}".format(self.node_name, key, topic))
                    self.publisher[key].publish(message)
            self._close_bag()
            return FlexGraspErrorCodes.SUCCESS
        else:
            return FlexGraspErrorCodes.FAILURE

    def publish_messages(self, messages, bag_path, bag_id):
        """write message(s) to a bag, and publish it/them"""
        success = self._open_bag(bag_path, bag_id, write=True)
        if success:
            overall_result = FlexGraspErrorCodes.SUCCESS

            # if the messages are provided as a dict, loop over the keys
            if isinstance(messages, dict):
                for key in messages:
                    result = self._publish_message(key, messages[key])
                    if result is not FlexGraspErrorCodes.SUCCESS:
                        overall_result = result

            # if only a single message is provided, check if the DataLogger is intialized with a single topic in mind
            else:
                keys = self.topics.keys()
                if len(keys) == 1:
                    key = keys[0]
                    overall_result = self._publish_message(key, messages)
                else:
                    rospy.logwarn("[{0}] Cannot publish message: unclear on which topic to publish!".format(self.node_name))
                    overall_result = FlexGraspErrorCodes.FAILURE
            self._close_bag()
            return overall_result
        else:
            return FlexGraspErrorCodes.FAILURE

    def _write_message(self, key, message):
        """Write received data in a rosbag"""
        my_type = self.types[key]
        if isinstance(message, my_type):
            rospy.logdebug("[{0}] Writing {1} to {2} bag".format(self.node_name, key, self.bag_name))
            self.bag.write(self.topics[key], message)
            return FlexGraspErrorCodes.SUCCESS
        else:
            message_type_name = type(message).__name__
            my_type_name = my_type.__name__
            rospy.logwarn("[{0}] Cannot write message to {1} bag: message of type {2} does not match specified type {3}"
                          .format(self.node_name, self.bag_name, message_type_name, my_type_name))
            return FlexGraspErrorCodes.FAILURE

    def _publish_message(self, key, message):
        my_type = self.types[key]
        if isinstance(message, my_type):
            rospy.logdebug("[{0}] Publishing {1}".format(self.node_name, key))
            self._write_message(key, message)
            self.publisher[key].publish(message)
            return FlexGraspErrorCodes.SUCCESS
        else:
            message_type_name = type(message).__name__
            my_type_name = my_type.__name__
            rospy.logwarn("[{0}] Cannot publish message: message of type {1} does not match specified type {2}"
                          .format(self.node_name, message_type_name, my_type_name))
            return FlexGraspErrorCodes.FAILURE

    def _open_bag(self, bag_path, bag_id, read=False, write=False):
        """open bag to read or write"""
        if self.bag is not None:
            rospy.logwarn("[{0}] Did not close previous {1} bag".format(self.node_name, self.bag_name))
            self._close_bag()

        bag_path = os.path.join(bag_path, bag_id)
        full_name = self.bag_name + '.bag'
        full_path = os.path.join(bag_path, full_name)

        rospy.logdebug("[{0}] Opening bag {1}".format(self.node_name, full_path))

        if not os.path.isdir(bag_path):
            rospy.loginfo("[{0}] New path, creating a new folder {1}".format(self.node_name, bag_path))
            os.makedirs(bag_path)

        if write and read:
            mode = 'a'
        elif read:
            mode = 'r'
            if not os.path.isfile(full_path):
                rospy.logwarn("[{0}] Cannot read file {1}: it does not exist on path {2}".format(self.node_name, full_name, bag_path))
                return False
        elif write:
            if os.path.isfile(full_path):
                rospy.logwarn("[{0}] Cannot write to file {1}: it already exists on path {2}".format(self.node_name, full_name, bag_path))
                return False
            mode = 'w'
        else:
            rospy.logwarn("[{0}] Cannot open bag: please specify whether you want to read or write!".format(self.node_name))
            return False

        self.bag = rosbag.Bag(full_path, mode)
        return True

    def _close_bag(self):
        """close bag, if not properly closed information will be lost!"""
        if self.bag is not None:
            rospy.logdebug("[{0}] Closing previous {1} bag".format(self.node_name, self.bag_name))
            self.bag.close()
            self.bag = None

    def reset(self):
        pass

    def wait_for_messages(self, time_out=1):
        pass

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