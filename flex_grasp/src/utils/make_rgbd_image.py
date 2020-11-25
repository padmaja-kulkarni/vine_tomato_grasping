#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 24 12:08:26 2020

@author: jelle
"""

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

path = os.path.abspath(os.path.join(os.path.dirname(__file__),".."))

# Start streaming
pipeline.start(config)


done = None

print("============ Press `y` to take a picture and `q` to quit")



try:
    while not(done):

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        
        k = cv2.waitKey(1)
        
        if k & 0xFF == ord('y'): #save on pressing 'y' 
            filename = str(time.ctime())
            np.save(path+'/Images/' + filename, images)
            cv2.imwrite(path+'/images/Color ' + filename + '.png', color_image)
            cv2.imwrite(path+'/images/Depth ' + filename + '.png', depth_colormap)
            print('============ Image saved as: ' + filename)
            
        elif k & 0xFF == ord('q'): #quit on pressing 'q' 
            done = True
            pipeline.stop()
            cv2.destroyAllWindows()
            break
        

finally:

    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
