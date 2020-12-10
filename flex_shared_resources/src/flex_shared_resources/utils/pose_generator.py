import rospy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped
from flex_shared_resources.utils.conversions import point_to_pose


class PoseGenerator(object):
    """Class for generating random poses within a slice of a disk"""

    def __init__(self, r_range, theta_range=[-np.pi/2, np.pi/2], x_min=0, z=0, frame=None, seed=0):
        """
        :param r_range: minimum and maximum radius of the disk
        :param theta_range: minimum and maximum angle of the disk
        :param x_min: minimum x value
        :param z: z value
        :param frame: frame used for pose stamped
        :param seed: seed used for random number generator
        """
        self.r_min = r_range[0]
        self.r_max = r_range[1]
        self.frame = frame
        self.theta_min = theta_range[0]
        self.theta_max = theta_range[1]

        self.x_min = x_min
        self.z = z
        random.seed(seed)

    def generate_pose_stamped(self):
        """Generate a random pose stamped"""
        if self.frame is None:
            print rospy.logerr("Cannot generate pose stamped, frame not defined")
        pose = self._generate()

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.frame
        return pose_stamped

    def generate_pose(self):
        """Generate a random pose"""
        return self._generate()


    def _generate(self, timeout=1):
        """Generate a random pose"""

        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < timeout) and not rospy.is_shutdown():
            theta = self.theta_min + random.random() * (self.theta_max - self.theta_min) # [rad]
            r = self.r_min + random.random() * (self.r_max - self.r_min)  # [m]
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            if x >= self.x_min:
                break

        orientation = (random.random() - 0.5) * 2 * np.pi

        goal_rpy = [0, 0, orientation]
        goal_xyz = [x, y, self.z]
        return point_to_pose(goal_xyz, goal_rpy)