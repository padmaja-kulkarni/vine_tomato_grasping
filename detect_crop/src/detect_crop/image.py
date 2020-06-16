# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import cv2

class Image(object):
    
    def __init__(self, data):
        
        self._data = data
        self._height= None
        self._height = None
        self._scale = 1
        
        self.update_shape()

    def rescale(self, width_desired):
        
        
        scale = width_desired/self._width
        new_width = scale * self._width
        new_height = scale * self._height        
        
        self._data = cv2.resize(self._data, (new_width, new_height), 
                                interpolation = cv2.INTER_AREA)
                                
        self.update_shape()
        
    def update_shape(self):
        
        shape = self._data.shape[:2]
        self._height = shape[0]
        self._width = shape[1]
        
        
    def crop(self):