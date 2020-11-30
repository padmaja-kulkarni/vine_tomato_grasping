#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import *
import random
import math


from flex_shared_resources.utils.conversions import list_to_orientation
from flex_shared_resources.utils.misc import generate_pose

model_ns = "product"


def delete_all_models():
    world_properties = get_world_properties()
    for model_name in world_properties.model_names:
        if model_ns in model_name:
            rospy.loginfo("Deleting model: %s", model_name)
            delete_model(model_name)


if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    rospy.wait_for_service("gazebo/get_world_properties")


    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    if rospy.has_param('px150/robot_base_frame'):
        reference_frame = rospy.get_param('px150/robot_base_frame')
    else:
        reference_frame = "world"

    with open("../models/tomato/model.sdf", "r") as f:
        product_xml = f.read()

    delete_all_models()


    x_min = 0.1
    r_range = [0.15, 0.23]
    heigth = 0.2

    n = 1

    for num in xrange(0, n):
        item_name = model_ns + "_{0}_0".format(num)
        rospy.loginfo("Spawning model: %s with respect to frame %s", item_name, reference_frame)
        pose = generate_pose(x_min, r_range, height=heigth)
        spawn_model(item_name, product_xml, "", pose, reference_frame)