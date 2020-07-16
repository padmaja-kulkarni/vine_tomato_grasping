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
from util import plot_circles
    
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

    
def detect_tomato(img_segment, imageRGB = None, blur_size = None, radius_min = None, 
                      radius_max = None, distance_min = None,
                      dp = None, param1 = None, param2 = None, ratio_threshold = None,
                      save = False, pwd = "", name = ""):
                     
    if imageRGB is None:                     
        imageRGB = img_segment
        
    # set dimensions
    dim = img_segment.shape
    minR = dim[1]/radius_min
    maxR = dim[1]/radius_max
    minDist = dim[1]/distance_min    

    # Hough requires a gradient, thus the image is blurred
    truss_blurred = cv2.GaussianBlur(img_segment, blur_size, 0)     
    
    # fit circles: [x, y, radius]
    circles = cv2.HoughCircles(truss_blurred, cv2.HOUGH_GRADIENT, 
                       dp, minDist, param1 = param1, 
                       param2 = param2, minRadius=minR, 
                       maxRadius=maxR) 

    if circles is None:
        centers = None
        radii = None
        com = None
            
    else:
        # swap columns [r, c] -> [x,y]
        centers =np.matrix(circles[0][:,0:2])
        radii = circles[0][:,2]
        
        # remove circles which do not overlapp with the tomato segment
        i_keep = find_overlapping_tomatoes(centers, radii, img_segment, ratio_threshold = ratio_threshold)
        centers = centers[i_keep, :]
        radii = radii[i_keep]
        
        com = (radii**2) * centers/(np.sum(radii**2))
    
    # visualize result
    thickness = 3
    tom_color = (150, 30, 0) 
    
    if save:
        plot_circles(imageRGB, centers, radii, pwd = pwd, name = name, 
                                     thickness = thickness, color=tom_color)
                                     
    return centers, radii, com

if __name__ == '__main__':
    pass