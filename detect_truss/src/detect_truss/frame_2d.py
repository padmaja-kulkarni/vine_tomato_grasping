# -*- coding: utf-8 -*-
"""
Created on Wed Jun 17 10:53:54 2020

@author: taeke
"""
import numpy as np



class Frame2D(object):
    
    def __init__(self, frame_id, parent_id, translation = (0,0), rotation = 0):
        self._parent_id = parent_id
        self._frame_id = frame_id
        self._translation = translation
        self._rotation = rotation
        
    def get(self):
        return self

    
if __name__ == '__main__':
    x = 10
    y = 20
    origin = 'original'
    
    rotation = np.deg2rad(10)
    translation = (0, 0)
    frame = Frame2D('rotated', origin, translation, rotation)    
    
