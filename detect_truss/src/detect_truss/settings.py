# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 15:05:05 2020

@author: taeke
"""

class Settings(object):

    def __init__(self):
                     
         #
        self.filterDiameterTom = 0
        self.filterDiameterPend = 0
        
        # detect tomatoes
        self.tomato_radius_min = 0
        self.tomato_radius_max = 0
        self.tomato_distance_min = 0
        self.dp = 0
        self.param1 = 0
        self.param2 = 0
        
        
        # detect junctions
        self.distance_threshold = None
        
        
    def set_default(self):
        
         filterDiameterTom = 11,
         filterDiameterPend = 5,
         tomato_radius_min = 8,
         tomato_radius_max = 2,
         tomato_distance_min = 5,
         dp = 5,
         param1 = 50,
         param2 = 150,
         distance_threshold = 10