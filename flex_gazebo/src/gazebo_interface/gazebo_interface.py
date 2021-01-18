#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import os
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties, SetModelState
from flex_shared_resources.utils.pose_generator import PoseGenerator

TIMEOUT = 1     # [s]

class GazeboInterface(object):
    """
    This class provides some useful Gazebo macros by wrapping ROS services.
    """
    truss_types = ['2d', '3d']

    def __init__(self, model_ns="model"):
        self.model_ns = model_ns + '_'
        self.model_name = self.model_ns + "{0}"

        rospy.logdebug("Waiting for gazebo services...")
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=TIMEOUT)
            rospy.wait_for_service("/gazebo/spawn_urdf_model",  timeout=TIMEOUT)
            rospy.wait_for_service("/gazebo/get_world_properties",  timeout=TIMEOUT)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=TIMEOUT)
        except rospy.exceptions.ROSException:
            rospy.logwarn("Unable to find gazebo services, did you launch Gazebo?")
            return

        rospy.logdebug("Found all services")

        self.delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_sdf_model_proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.spawn_urdf_model_proxy = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.get_world_properties_proxy = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.set_model_state_proxy = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # z > 0 otherwise truss falls trought table
        self.pose_generator = PoseGenerator(r_range=[0.15, 0.23], x_min=0.18,  z=0.03)

        if rospy.has_param('robot_base_frame'):
            self.reference_frame = rospy.get_param('robot_base_frame')
        else:
            self.reference_frame = "world"

        self.xml_models = {}
        for truss_type in self.truss_types:
            self.xml_models[truss_type] = self.load_sdf(truss_type)

    def delete_all_models(self):
        """
            Deletes all models present in Gazebo world
        """
        success = True
        model_names = self._get_model_names()

        for model_name in model_names:
            rospy.loginfo("Deleting model: %s", model_name)

            try:
                result = self.delete_model_proxy(model_name)
            except Exception as exc:
                rospy.loginfo("Service did not process request: " + str(exc))
                continue

            if result.success:
                rospy.loginfo("Successfully deleted model: %s", model_name)
            else:
                success = False
                rospy.logwarn(result.status_message)

        return success

    def set_model_pose(self):
        """
            Randomize the pose of all models present in Gazebo world
        """
        success = True
        model_names = self._get_model_names()

        for model_name in model_names:

            pose = self.pose_generator.generate_pose()
            state_msg = ModelState(model_name=model_name, pose=pose, reference_frame=self.reference_frame)

            try:
                result = self.set_model_state_proxy(state_msg)
            except Exception as exc:
                rospy.loginfo("Service did not process request: " + str(exc))

            if result.success:
                rospy.loginfo("Successfully randomized pose of model: %s", model_name)
            else:
                success = False
                rospy.logwarn(result.status_message)

        return success

    def spawn_model(self, truss_type):
        """
            Spawns an sdf model to Gazebo, with an unique name
        """
        success = True

        if truss_type not in self.truss_types:
            rospy.logwarn("Cannot spawn model of type {0}, unknown type! Available types are {1}".format(truss_type, self.truss_types))
            return False

        new_id = self._generate_unique_id()
        item_name = self.model_name.format(new_id)
        xml_model = self.xml_models[truss_type]
        pose = self.pose_generator.generate_pose()

        try:
            result = self.spawn_sdf_model_proxy(item_name, xml_model, "", pose, self.reference_frame)
        except Exception as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        if result.success:
            rospy.loginfo("Successfully spawned sdf model {0} with respect to {1} frame".format(item_name, self.reference_frame))
        else:
            success = False
            rospy.logwarn(result.status_message)

        return success

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

    def spawn_urdf_model(self):
        """
            Spawns an urdf model to gazebo, with an unique name
        """
        new_id = self._generate_unique_id()
        item_name = self.model_name.format(new_id)

        rospy.loginfo("Spawning urdf model: {0} with respect to frame {1}".format(item_name, self.reference_frame))
        pose = self.pose_generator.generate_pose()
        self.spawn_urdf_model_proxy(item_name, self.model_xml_urdf, "", pose, self.reference_frame)

    def _generate_unique_id(self):
        """ generate a new id which is the lowest value not already present in the gazebo world"""
        used_ids = self._get_model_ids()

        if not used_ids:
            return 1
        else:
            # https://stackoverflow.com/a/28178803
            return next(i for i, e in enumerate(sorted(used_ids) + [None], 1) if i != e)

    def _get_model_ids(self):
        """Gets al model ids from the Gazebo world"""
        model_ids = []
        model_names = self._get_model_names()
        for model_name in model_names:
            split_model_name = model_name.split(self.model_ns)
            model_ids.append(int(split_model_name[1]))
        return model_ids

    def _get_model_names(self):
        """Gets al model names from the Gazebo world which start with the model name space followed by an integer"""
        world_properties = self.get_world_properties_proxy()
        model_names = []
        for model_name in world_properties.model_names:
            split_model_name = model_name.split(self.model_ns)
            if len(split_model_name) == 2:
                if (split_model_name[0] == '') and split_model_name[1].isdigit():
                    model_names.append(model_name)

        return model_names
