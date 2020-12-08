import rospy
from flex_grasp.msg import FlexGraspErrorCodes

class StateMachineInput(object):


    def __init__(self, node_name):
        self.NODE_NAME = node_name
        self.command = None
        self.model_type = None

        rospy.Subscriber("~e_in", String, self.e_in_cb)
        rospy.Subscriber("object_features", Truss, self.object_features_cb)
        rospy.Subscriber("peduncle_height", Float32, self.peduncle_height_cb)

        self.pub_e_out = rospy.Publisher("~e_out", FlexGraspErrorCodes, queue_size=10, latch=True)