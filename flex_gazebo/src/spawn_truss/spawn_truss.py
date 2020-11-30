#!/usr/bin/env python

import rospy
import os
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from flex_shared_resources.utils.misc import generate_pose



class ModelSpawner(object):
    service_timeout = 1  # [s]

    def __init__(self, model_ns="model"):
        self.model_ns = model_ns + '_'
        self.model_name = self.model_ns + "{0}"

        rospy.logdebug("Waiting for gazebo services...")
        rospy.wait_for_service("/gazebo/delete_model", timeout=self.service_timeout)
        rospy.wait_for_service("/gazebo/spawn_urdf_model",  timeout=self.service_timeout)
        rospy.wait_for_service("/gazebo/get_world_properties",  timeout=self.service_timeout)
        rospy.logdebug("Found all services")

        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

        if rospy.has_param('robot_base_frame'):
            self.reference_frame = rospy.get_param('robot_base_frame')
        else:
            self.reference_frame = "world"

        pwd = os.path.dirname(os.path.abspath(__file__))
        full_pwd = os.path.join(pwd, '..', '..', 'models', 'tomato', 'model.sdf')
        with open(full_pwd, "r") as f:
            self.product_xml = f.read()

        self.spawn_x_min = 0.18  # [m]
        self.spawn_r_range = [0.15, 0.23]  # [m]
        self.spawn_heigth = 0.1  # [m]

    def delete_all_models(self):
        """
            deletes all models in Gazebo which start with the model name space followed by an integer
        """

        world_properties = self.get_world_properties()
        for model_name in world_properties.model_names:
            split_model_name = model_name.split(self.model_ns)
            if len(split_model_name) == 2:
                if (split_model_name[0] == '') and split_model_name[1].isdigit():
                    rospy.loginfo("Deleting model: %s", model_name)
                    self.delete_model(model_name)

    def add_model(self):
        """
            adds a model to Gazebo, with a unique name
        """

        world_properties = self.get_world_properties()

        used_id = []
        for model_name in world_properties.model_names:
            split_model_name = model_name.split(self.model_ns)
            if len(split_model_name) == 2:
                if (split_model_name[0] == '') and split_model_name[1].isdigit():
                    used_id.append(int(split_model_name[1]))

        # make the new id the lowest value which is not already present in the used_id list
        if not used_id:
            new_id = 1
        else:
            # https://stackoverflow.com/a/28178803
            new_id = next(i for i, e in enumerate(sorted(used_id) + [None], 1) if i != e)

        item_name = self.model_name.format(new_id)
        rospy.loginfo("Spawning model: %s with respect to frame %s", item_name, self.reference_frame)
        pose = generate_pose(self.spawn_x_min, self.spawn_r_range, height=self.spawn_heigth)
        self.spawn_model(item_name, self.product_xml, "", pose, self.reference_frame)


def main():
    model_spawner = ModelSpawner()
    model_spawner.delete_all_models()
    model_spawner.add_model()


if __name__ == '__main__':
    main()