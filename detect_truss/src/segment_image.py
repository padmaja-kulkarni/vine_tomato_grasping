# -*- coding: utf-8 -*-
"""
Created on Fri May 22 12:04:59 2020

@author: taeke
"""

## imports ##
import os # os.sep
import cv2

# custom functions
from detect_truss.util import plot_segments
from detect_truss.util import load_rgb
from detect_truss.util import make_dirs

from detect_truss.segment_image import segment_truss

# ls | cat -n | while read n f; do mv "$f" `printf "%03d.png" $n`; done
if __name__ == '__main__':

    N = 2         # tomato file to load
    extension = ".png"
    dataset ="failures" # "tomato_rot" #  "depth_blue" #  
    save = True

    pwd_current = os.path.dirname(__file__)
    pwd_data = os.path.join(pwd_current, "data", dataset)
    pwd_results = os.path.join(pwd_current, "results", dataset, "02_segment")
    
    make_dirs(pwd_results)
    
    count = 0
    
    for i_tomato in range(1, N):
    
        tomato_ID = str(i_tomato).zfill(3)
        tomato_name = tomato_ID
        file_name = tomato_name + extension
        

        img_rgb = load_rgb(pwd_data, file_name, horizontal = True)
    
        # color spaces
        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
        img_hue = img_hsv[:, :, 0] # hue
        
        img_lab = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2LAB)    
        img_a = img_lab[:, :, 1]
        background, tomato, peduncle = segment_truss(img_hue,
                                                     img_a=img_a,
                                                     save = save, 
                                                     name = tomato_name, 
                                                     pwd = pwd_results) 
        
        # VISUALIZE
        name = tomato_ID + "_img"
        plot_segments(img_rgb, background, tomato, peduncle, 
                      name = name, pwd = pwd_results)
      
        count = count + 1
        print("completed image %d out of %d" %(count, N))