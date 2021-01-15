import os
import rospy
from std_msgs.msg import String

class ExperimentInfo(object):
    """Simple class to store experiment info"""
    def __init__(self, node_name, namespace=None, id=None, path=None):

        self.node_name = node_name
        self.path = path
        self.id = id

        if namespace is None:
            experiment_id_topic = 'experiment_id'
            experiment_path_topic = 'experiment_pwd'
        else:
            experiment_id_topic = '/' + namespace + '/' + 'experiment_id'
            experiment_path_topic = '/' + namespace + '/' + 'experiment_pwd'

        rospy.Subscriber(experiment_id_topic, String, self.experiment_id_cb)
        rospy.Subscriber(experiment_path_topic, String, self.experiment_pwd_cb)

    ## callbacks
    def experiment_id_cb(self, msg):
        """callback"""
        new_id = msg.data
        if self.id != new_id:
            self.id = new_id
            rospy.logdebug("[{0}] Storing results with experiment id {1}".format(self.node_name, new_id))

    def experiment_pwd_cb(self, msg):
        """callback to update the data path"""
        new_path = msg.data
        if self.path != new_path:
            self.path = new_path
            rospy.logdebug("[{0}] Storing results in folder {1}".format(self.node_name, new_path))
