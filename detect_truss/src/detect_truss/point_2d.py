# -*- coding: utf-8 -*-
"""
Created on Wed Jun 17 10:43:57 2020

@author: taeke
"""
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import tf2_ros
from image import Image, image_rotate
from moveit_commander import MoveItCommanderException
import tf2_geometry_msgs


from tf.transformations import quaternion_from_euler


from util import add_circles
from util import translation_rot2or

def list_to_orientation(orientation_list):
          
    if len(orientation_list) == 3:
        quat_list = quaternion_from_euler(orientation_list[0], orientation_list[1], orientation_list[2])
    elif len(orientation_list) == 4:
        quat_list = orientation_list
    else: 
        raise MoveItCommanderException("Expected orinetation list containing 3 (x, y, z) or 4 (x, y, z, w) elements")
    
    orientation_msg = Quaternion()
    orientation_msg.x = quat_list[0]
    orientation_msg.y = quat_list[1]
    orientation_msg.z = quat_list[2]
    orientation_msg.w = quat_list[3]
    
    return orientation_msg

def make_2d_transform(from_frame_id, to_frame_id, xy = (0,0), angle = 0):
    
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time(0)
    transform_stamped.header.frame_id = from_frame_id
    transform_stamped.child_frame_id = to_frame_id
    transform_stamped.transform.translation.x = xy[0]
    transform_stamped.transform.translation.y = xy[1]
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation = list_to_orientation((0,0,angle))
    return transform_stamped

def make_2d_point(frame_id, xy = None):

    pose_stamped = PoseStamped()    
    
    if xy is None:
        xy = (0,0)
            
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose.position.x = xy[0]
    pose_stamped.pose.position.y = xy[1]
    return pose_stamped

def make_2d_points(frame_id, XY):
    pose_stamped_list = []
    for xy in XY:
        pose_stamped_list.append(make_2d_point(frame_id, xy))

    return pose_stamped_list

class Point2D(object):
    
    def __init__(self, frame_id, xy = None):
        
        if xy is None:
            xy = (0,0)
        
        self._point = make_2d_point(frame_id, xy)        
        self._frame_id = frame_id
        
        
    def get_point(self, transform = None):
        
        
        if transform is None:
            return point_2d
        
        return tf2_geometry_msgs.do_transform_pose(point_2d, point_transform)
        
        
class Transform2D(object):
    
    def __init__(self):

        self._buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))        
        
    def add_transform(self, from_frame_id, to_frame_id, translation, angle):
        
        t = make_2d_transform(from_frame_id, to_frame_id, translation , angle)
        self._buffer_core.set_transform(t, "default_authority")
        
    def _get_frames_as_string(self):  
        
        return self._buffer_core.all_frames_as_string()
    
if __name__ == '__main__':
    
    
    
    # params
    angle = np.deg2rad(10) 
    frame_id = 'original'    
    xy = (200,400)
    height = 1000
    width = 2000
    dim = (height, width)
    brightness = 100
    
    # init data
    data = brightness*np.ones((height,width,3), dtype=np.uint8)
    point_2d = make_2d_point(frame_id, xy)
    
    # init buffer
    buffer_core = tf2_ros.BufferCore(rospy.Duration(10.0))

    # create transforms
    translation = translation_rot2or(dim, -angle)
    transform = make_2d_transform('rotate', 'original', translation , -angle)
    buffer_core.set_transform(transform, "default_authority")   
    

    # transform point
    point_transform = buffer_core.lookup_transform_core('original', 'rotate', rospy.Time(0)) 
    point_new = tf2_geometry_msgs.do_transform_pose(point_2d, point_transform)    
    
    
    # visualization
    centers = np.array(xy, ndmin = 2)    
    image = Image(add_circles(data, centers, radii = 20, color = (100, 0, 0)))   
    image.show()    
    
    image2 = image_rotate(image, -angle)  
    xy_new = (point_new.pose.position.x, point_new.pose.position.y)
    centers_new = np.array(xy_new, ndmin = 2)
    image = Image(add_circles(image2.get_data(), centers_new, radii = 40, color = (0, 100, 0)))   
    image.show()
    
    
    # frames = buffer_core.all_frames_as_string()