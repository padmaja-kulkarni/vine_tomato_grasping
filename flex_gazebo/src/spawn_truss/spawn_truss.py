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
        self.spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.spawn_urdf_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

        if rospy.has_param('robot_base_frame'):
            self.reference_frame = rospy.get_param('robot_base_frame')
        else:
            self.reference_frame = "world"

        self.truss_types = ['2d', '3d']
        self.xml_models = {}
        for truss_type in self.truss_types:
            self.xml_models[truss_type] = self.load_sdf(truss_type)


        self.spawn_x_min = 0.18  # [m]
        self.spawn_r_range = [0.15, 0.23]  # [m]
        self.spawn_height = 0.1  # [m]

    def load_sdf(self, truss_type):

        if truss_type in self.truss_types:
            rospy.loginfo('[SPAWN TRUSS] loading sdf of %s truss type', truss_type)
        else:
            rospy.logwarn('[SPAWN TRUSS] unknown truss type %s: cannot load sdf', truss_type)
            return

        pwd = os.path.dirname(os.path.abspath(__file__))
        full_pwd = os.path.join(pwd, '..', '..', 'models', truss_type + '_truss', 'model.sdf')
        with open(full_pwd, "r") as f:
            model_xml = f.read()

        return model_xml

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

    def spawn_3d_model(self):
        self.add_sdf_model('3d')

    def spawn_2d_model(self):
        self.add_sdf_model('2d')

    def add_sdf_model(self, truss_type=None):
        """
            adds a model to Gazebo, with a unique name
        """

        new_id = self.get_unique_id()
        item_name = self.model_name.format(new_id)
        xml_model = self.xml_models[truss_type]

        rospy.loginfo("Spawning sdf model: %s with respect to frame %s", item_name, self.reference_frame)
        pose = generate_pose(self.spawn_x_min, self.spawn_r_range, height=self.spawn_height)
        self.spawn_sdf_model(item_name, xml_model, "", pose, self.reference_frame)

    def add_urdf_model(self):
        new_id = self.get_unique_id()
        item_name = self.model_name.format(new_id)

        rospy.loginfo("Spawning urdf model: %s with respect to frame %s", item_name, self.reference_frame)
        pose = generate_pose(self.spawn_x_min, self.spawn_r_range, height=self.spawn_height)
        self.spawn_urdf_model(item_name, self.model_xml_urdf, "", pose, self.reference_frame)

    def get_unique_id(self):
        world_properties = self.get_world_properties()

        used_id = []
        for model_name in world_properties.model_names:
            split_model_name = model_name.split(self.model_ns)
            if len(split_model_name) == 2:
                if (split_model_name[0] == '') and split_model_name[1].isdigit():
                    used_id.append(int(split_model_name[1]))

        # make the new id the lowest value which is not already present in the used_id list
        if not used_id:
            return 1
        else:
            # https://stackoverflow.com/a/28178803
            return next(i for i, e in enumerate(sorted(used_id) + [None], 1) if i != e)

def main():
    rospy.init_node("move_robot",
                    anonymous=True,
                    log_level=rospy.DEBUG)

    model_spawner = ModelSpawner()
    model_spawner.delete_all_models()
    model_spawner.spawn_3d_model()


if __name__ == '__main__':
    main()
