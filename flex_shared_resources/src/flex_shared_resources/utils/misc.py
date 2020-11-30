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


def call_service(service_name, service_msg_type, args, service_timeout=1):
    """
    Wait for the service called service_name
    Then call the service with args

    :param service_name:
    :param service_msg_type:
    :param args: Tuple of arguments
    :param service_timeout:
    :raises Exception: Timeout during waiting of services
    :return: Response
    """

    # Connect to service
    try:
        rospy.wait_for_service(service_name, service_timeout)
    except rospy.ROSException, e:
        raise e

    # Call service
    try:
        service = rospy.ServiceProxy(service_name, service_msg_type)
        response = service(*args)
        return response
    except rospy.ServiceException, e:
        raise e
