import os
import rospy
from std_msgs.msg import String

class ExperimentInfo(object):
    """Simple class to store experiment info"""
    def __init__(self, node_name):

        self.node_name = node_name
        self.path = os.path.join(os.getcwd(), 'thesis_data', 'default')
        self.id = None

        rospy.Subscriber("experiment_id", String, self.experiment_id_cb)
        rospy.Subscriber("experiment_pwd", String, self.experiment_pwd_cb)

    ## callbacks
    def experiment_id_cb(self, msg):
        """callback"""
        new_id = msg.data
        if self.id != new_id:
            self.id = new_id
            rospy.loginfo("[{0}] Storing results with experiment id {1}".format(self.node_name, new_id))

    def experiment_pwd_cb(self, msg):
        """callback to update the data path"""
        new_path = msg.data
        if self.path != new_path:
            self.path = new_path
            rospy.loginfo("[{0}] Storing results in folder {1}".format(self.node_name, new_path))
