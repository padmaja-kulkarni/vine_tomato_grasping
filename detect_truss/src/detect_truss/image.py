#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt

from skimage.measure import label, regionprops
from skimage.transform import rotate

import warnings


def check_dimensions(image1, image2):
    b_height = image1._height == image2._height
    b_width = image1._width == image2._width
    b_angle = image1._angle == image2._angle
    b_scale = image1._scale == image2._scale

    return b_height and b_width and b_angle and b_scale

def add(image1, image2):
    
    if check_dimensions:
        
        image_new = image1.copy()        
        image_new.data = cv2.bitwise_or(image1.data, image2.data) 
        return image_new
    else:
        return None
    
    
def image_crop(image, angle=None, bbox=None):
    """returns a new image, rotated by angle in radians and cropped by the boundingbox"""
    image_new = image.copy()
    image_new.crop(angle=angle, bbox=bbox)
    return image_new


def image_rotate(image, angle):
    """returns a new image, rotated by angle in radians"""
    image_new = image.copy()
    image_new.rotate(angle)
    return image_new


def image_cut(image, bbox):
    """returns a new image, cut at the boundingbox"""
    image_new = image.copy()
    image_new.cut(bbox)
    return image_new

def compute_bbox(image):
    return cv2.boundingRect(image)

class Image(object):
    
    def __init__(self, data):
        self.data = data
        self.dtype = data.dtype
        self.value_max = np.iinfo(self.dtype).max

    def rescale(self, width_desired):
        shape = self.data.shape[:2] # (height, width)
        scale = float(width_desired)/float(shape[1])
        new_shape = (int(scale * shape[1]), int(scale * shape[0])) # [width, height]
        
        self.data = cv2.resize(self.data, new_shape, 
                               interpolation = cv2.INTER_AREA)
                               
        return scale
        
    def open_close(self, kernel):
        image_open = cv2.morphologyEx(self.data, cv2.MORPH_OPEN, kernel)
        self.data = cv2.morphologyEx(image_open, cv2.MORPH_CLOSE, kernel)        

    def close_open(self, kernel):
        image_close = cv2.morphologyEx(self.data, cv2.MORPH_CLOSE, kernel)
        self.data = cv2.morphologyEx(image_close, cv2.MORPH_OPEN, kernel)       
        
    def is_empty(self):
        return np.all((self.data == 0))        
        
    def rotate(self, angle):

        # rotate returns a float in range [0, 1], this needs to be converted
        image_rotate = rotate(self.data, np.rad2deg(angle), resize=True) 
        self.data = (self.value_max*image_rotate).astype(self.dtype, copy=False)
        return self

    def cut(self, bbox):
        x = bbox[0]
        y = bbox[1]
        w = bbox[2]
        h = bbox[3]        
        self.data = self.data[y:y+h, x:x+w]
        return self

    def crop(self, angle, bbox):
        
        self.rotate(angle)
        self.cut(bbox)
        return self        

    def copy(self):
        image_new = Image(self.data.copy())
        return image_new
        
    def show(self):
        plt.imshow(self.data)
        plt.axis('off')

    def compute_angle(self):
        """returns the angle in radians based on the Image data
        """

        regions = regionprops(label(self.data), coordinates='xy')

        if len(regions) > 1:
            warnings.warn("Multiple regions found!")

        return regions[0].orientation

