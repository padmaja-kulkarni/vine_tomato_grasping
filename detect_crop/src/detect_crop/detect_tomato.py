# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 14:21:38 2020

@author: taeke
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 08:52:33 2020

@author: taeke
"""


## imports ##
import cv2
import numpy as np

# custom functions
from util import plot_features

def detect_tomato(img_segment, settings, img_rgb = None,
                      save = False, pwd = "", name = ""):
                     
    if img_rgb is None:                     
        img_rgb = img_segment
        

    # set dimensions
    dim = img_segment.shape
    min_radius = dim[1]/settings['radius_min']
    max_radius = dim[1]/settings['radius_max']
    min_distance = dim[1]/settings['distance_min']    

    # Hough requires a gradient, thus the image is blurred
    truss_blurred = cv2.GaussianBlur(img_segment, settings['blur_size'], 0)     
    
    # fit circles: [x, y, radius]
    circles = cv2.HoughCircles(truss_blurred, 
                               cv2.HOUGH_GRADIENT, 
                               settings['dp'], 
                               min_distance, 
                               param1 = settings['param1'], 
                               param2 = settings['param2'], 
                               minRadius = min_radius, 
                               maxRadius=max_radius) 

    if circles is None:
        centers = None
        radii = None
        com = None
            
    else:
        # swap columns [r, c] -> [x,y]
        centers =np.matrix(circles[0][:,0:2])
        radii = circles[0][:,2]
        
        # remove circles which do not overlapp with the tomato segment
        i_keep = find_overlapping_tomatoes(centers, 
                                           radii, 
                                           img_segment, 
                                           ratio_threshold = settings['ratio_threshold'])
        centers = centers[i_keep, :]
        radii = radii[i_keep]
        
        com = (radii**2) * centers/(np.sum(radii**2))
    
    # visualize result
    thickness = 1
    tom_color = (150, 30, 0) 
    
       
    if save:
        tomato = {'centers': centers, 'radii': radii} 
        plot_features(img_rgb, tomato, pwd = pwd, file_name=name, thickness = thickness)
#        plot_circles(img_rgb, centers, radii, pwd = pwd, name = name, 
#                                     thickness = thickness, color=tom_color)
                                     
    return centers, radii, com

def set_detect_tomato_settings(blur_size = (3,3),
                               radius_min = 8,
                               radius_max = 4,
                               distance_min = 4, # = tomato_radius_max
                               dp = 4,
                               param1 = 20,
                               param2 = 50,
                               ratio_threshold = 0.5):   
    
    settings = {}
    settings['blur_size'] = blur_size  
    settings['radius_min'] = radius_min  
    settings['radius_max'] = radius_max
    settings['distance_min'] = distance_min
    settings['dp'] = dp
    settings['param1'] = param1
    settings['param2'] = param2
    settings['ratio_threshold'] = ratio_threshold
    return settings

def find_overlapping_tomatoes(centers, radii, img_segment, ratio_threshold = 0.5):
        
    
    iKeep = []
    N = centers.shape[0]
    for i in range(0, N):
     
        image_empty = np.zeros(img_segment.shape, dtype=np.uint8)
        mask = cv2.circle(image_empty,(centers[i,0], centers[i,1]), radii[i], 255, -1)            
        
        res = cv2.bitwise_and(img_segment, mask)
        pixels = np.sum(res == 255)
        total = np.pi*radii[i]**2
        ratio = pixels/total
        if ratio > ratio_threshold:
            iKeep.append(i)
            
    return iKeep

if __name__ == '__main__':
    print("This file has no main!")