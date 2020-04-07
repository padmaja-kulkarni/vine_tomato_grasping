#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  3 09:30:31 2020

@author: taeke
"""

import rospy

import pickle

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class AnalyzePointCloud(object):

    def __init__(self):

        rospy.init_node("Analyze_Point_Cloud",
                        anonymous=True, log_level=rospy.DEBUG)

        self.point_cloud = None
        self.points = None

        rospy.Subscriber("/realsense_plugin/camera/depth/points", PointCloud2, self.point_cloud_cb)


    def point_cloud_cb(self, msg):
        if self.point_cloud is None:
            self.point_cloud = msg
            rospy.logdebug("Received new point cloud message")

    def get_points(self):
        rospy.logdebug("Getting Points")
        self.points = list(pc2.read_points(self.point_cloud, skip_nans=True, field_names = ("x", "y", "z")))
        

    def save_points(self, filename =  "points"):
        return self.save(self.points, filename =  "points")


    def save_point_cloud(self, filename =  "pointcloud"):
        return self.save(self.point_cloud, filename =  "pointcloud")


    def save(self, data, filename =  "file"):
        rospy.logdebug("Saving %s...", filename)
        outfile = file(filename, 'wb')
        pickle.dump(data, outfile)
        outfile.close

        rospy.logdebug("Saving %s done!", filename)
        return True

def main():
    try:
        analyze_point_cloud = AnalyzePointCloud()
        rate = rospy.Rate(10)
        while not rospy.core.is_shutdown():
            if analyze_point_cloud.point_cloud is not None:
                # analyze_point_cloud.save_point_cloud()
                analyze_point_cloud.get_points()
                analyze_point_cloud.save_points()
                break


            rate.sleep()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
