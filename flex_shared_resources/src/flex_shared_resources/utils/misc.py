import rospy


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
