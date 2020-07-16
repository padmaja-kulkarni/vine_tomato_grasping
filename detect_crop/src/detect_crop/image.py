# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import cv2
import numpy as np
from util import romove_blobs
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
        image_new._data = cv2.bitwise_or(image1._data, image2._data) 
        return image_new
    else:
        return None

def compute_angle(image):
    
    regions = regionprops(label(image), coordinates='xy')
        
    if len(regions) > 1: 
        warnings.warn("Multiple regions found!")
        
    angle = regions[0].orientation # np.rad2deg()
    return angle
    
    
def image_crop(image, angle = None, bbox = None):
    image_new = image.copy()
    image_new.crop(angle = angle, bbox = bbox)
    return image_new
    
def image_rotate(image, angle):
    
    image_new = image.copy()
    image_new.rotate(angle)
    return image_new

def compute_bbox(image):
    return cv2.boundingRect(image)

class Image(object):
    
    def __init__(self, data):
        self._sub_image = []
        self._data = data
        self._update_shape()
        self._type = data.dtype.type
        self._data_max = np.iinfo(self._type).max
        self._update_shape()
        
        self._scale = 1
        self._angle = 0

    def rescale(self, width_desired):
        scale = float(width_desired)/float(self._width)
        new_width = int(scale * self._width)
        new_height = int(scale * self._height)    
        
        self._data = cv2.resize(self._data, (new_width, new_height), 
                                interpolation = cv2.INTER_AREA)
                                
        self._update_shape()
        
        
    def _update_shape(self):
        shape = self._data.shape[:2]
        self._height = shape[0]
        self._width = shape[1]
        
        
    def open_close(self, kernel):
        self._data = cv2.morphologyEx(cv2.morphologyEx(self._data, 
                                                       cv2.MORPH_OPEN, 
                                                       kernel),
                                       cv2.MORPH_CLOSE, kernel)        
        
        
    def close_open(self, kernel):
        self._data = cv2.morphologyEx(cv2.morphologyEx(self._data, 
                                                       cv2.MORPH_CLOSE, 
                                                       kernel), 
                                       cv2.MORPH_OPEN, kernel)
        
    def remove_blobs(self):
        self._data = romove_blobs(self._data, self._data_max)        
        
    def is_empty(self):
        return np.all((self._data == 0))        
        
    def get_data(self):
        return self._data    
        
    def rotate(self, angle):

        # rotate
        self._data= self._type(self._data_max*rotate(self._data, 
                                                     np.rad2deg(angle), 
                                                        resize=True))        
        self._angle = angle
        self._update_shape()
        
    def cut(self, box):

        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]        
        
        self._data = self._data[y:y+h, x:x+w]
        
    def crop(self, angle = None, bbox = None):
        if angle == None:
            angle = compute_angle(self._data)
            
        if bbox == None:
            bbox = compute_bbox(self._data)
        
        self.rotate(angle)
        self.cut(bbox)
        return self        
        
    def copy(self):
        image_new = Image(self._data.copy())
        image_new._scale = self._scale
        image_new._angle = self._angle
        return image_new
        
    def show(self):
        plt.imshow(self._data)
        plt.axis('off')
        
    def get_dimensions(self):
        
        return (self._height, self._width)