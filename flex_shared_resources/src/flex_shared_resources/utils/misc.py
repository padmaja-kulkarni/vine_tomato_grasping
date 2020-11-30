import numpy as np
import random
import rospy

from flex_shared_resources.utils.conversions import point_to_pose

def generate_pose(x_min, r_range, theta_range=[-np.pi/2, np.pi/2], height=0):
    """
    generates a random pose which lies on a half circle with a radius in the given range:

    """

    theta_min = theta_range[0]
    theta_max = theta_range[1]

    r_min = r_range[0]
    r_max = r_range[1]
    x = 0.0
    while x <= x_min:
        theta = theta_min + random.random() * (theta_max - theta_min)  # [rad]
        r = r_min + random.random() * (r_max - r_min)  # [m]

        x = r * np.cos(theta)
        y = r * np.sin(theta)

    # TODO: combinening two uniform distributions, does not give a uniform distribution!
    orientation = random.random() * 2*np.pi  # [rad]


    pose_rpy = [0, 0, orientation]
    pose_xyz = [x, y, height]
    pose = point_to_pose(pose_xyz, pose_rpy)
    return pose