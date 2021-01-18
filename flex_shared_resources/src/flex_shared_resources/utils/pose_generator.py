import rospy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped
from flex_shared_resources.utils.conversions import point_to_pose

class PoseGenerator(object):
    """Class for generating random poses within a slice of a disk"""

    def __init__(self, r_range, theta_range=None, x_min=0, z=0, frame=None, seed=0):
        """
        :param r_range: minimum and maximum radius of the disk
        :param theta_range: minimum and maximum angle of the disk
        :param x_min: minimum x value
        :param z: z value
        :param frame: frame used for pose stamped
        :param seed: seed used for random number generator
        """
        if theta_range is None:
            theta_range = [-np.pi / 2, np.pi / 2]

        self.r_min = r_range[0]
        self.r_max = r_range[1]
        self.frame = frame
        self.theta_min = theta_range[0]
        self.theta_max = theta_range[1]

        self.x_min = x_min
        self.z = z
        random.seed(seed)

    def generate_pose_stamped(self, seed=None):
        """Generate a random pose stamped"""
        if self.frame is None:
            print rospy.logerr("Cannot generate pose stamped, frame not defined")
        pose = self._generate(seed=seed)

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.frame
        return pose_stamped

    def generate_pose(self, seed=None):
        """Generate a random pose"""
        return self._generate(seed=seed)

    def _generate(self, timeout=1, seed=None):
        """Generate a random pose"""

        if seed is not None:
            random.seed(seed)

        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < timeout) and not rospy.is_shutdown():
            theta = self.theta_min + random.random() * (self.theta_max - self.theta_min)  # [rad]
            r = self.r_min + random.random() * (self.r_max - self.r_min)  # [m]
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            if x >= self.x_min:
                break
        orientation = (random.random() - 0.5) * 2 * np.pi

        goal_rpy = [0, 0, orientation]
        goal_xyz = [x, y, self.z]
        return point_to_pose(goal_xyz, goal_rpy)


def main():
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    from tf.transformations import euler_from_quaternion
    from utils.conversions import orientation_to_list
    node_name = 'transform_pose'
    robot_base_frame = 'px150/base_link'

    rospy.init_node(node_name, anonymous=True, log_level=rospy.INFO)
    pose_generator = PoseGenerator(r_range=[0.15, 0.23], x_min=0.17, frame=robot_base_frame, seed=node_name)

    pose_stamped_list = []
    for i in range(1, 21):
        experiment_id = str(i).zfill(3)
        pose_generator.z = 0
        pose_stamped = pose_generator.generate_pose_stamped(seed=int(int(experiment_id)*np.pi*10**2))
        pose_stamped_list.append(pose_stamped)

    plt.figure()
    for pose_stamped in pose_stamped_list:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        theta = euler_from_quaternion(orientation_to_list(pose_stamped.pose.orientation))[2]
        t = mpl.markers.MarkerStyle(marker='|')
        t._transform = t.get_transform().rotate_deg(np.rad2deg(theta))
        plt.scatter(y, x, marker=t, c='blue', s=300)

    plt.xlim((-0.23, 0.23))
    plt.ylim((0.17, 0.23))
    plt.gca().axis('equal')
    plt.show()

if __name__ == '__main__':
    main()
