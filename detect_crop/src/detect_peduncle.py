# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:16:42 2020

@author: taeke
"""

import os
from detect_crop.util import make_dirs, load_rgb
from detect_crop.util import change_brightness

import time

from detect_crop.ProcessImage import ProcessImage 
from detect_crop.detect_peduncle import detect_peduncle

if __name__ == '__main__':
    
    #%% init
     #  48 #        # tomato file to load
    nDigits = 3
    
    pwd_current = os.path.dirname(__file__)
    dataset ="real_blue" #  "empty" # "artificial" # 
    
    pwd_data = os.path.join(pwd_current, "data", dataset)
    pwd_results = os.path.join(pwd_current, "results", dataset, "detect_peduncle")
    
    make_dirs(pwd_results)

    brightness = 0.85
    
    for iTomato in range(1,2):
        
        tomato_name = str(iTomato).zfill(nDigits)
        file_name = tomato_name + ".png"
        
        img_rgb = load_rgb(pwd_data, file_name, horizontal = True)

        
        image = ProcessImage(use_truss = True,
                                         name = tomato_name,
                                         pwd = pwd_results,
                                         save = False)
        
        image.add_image(img_rgb)    
        
        image.color_space()
        image.segment_truss()
        image.filter_img()
        image.rotate_cut_img()
        image.detect_tomatoes()
    
        distance_threshold = 10
        
        segment_img = image.get_segmented_image(local = True)   
        peduncle_img = image.get_peduncle_image(local = True)
        segment_img_bright = change_brightness(segment_img, brightness)
              

        start_time = time.time()
        skeleton_img, branch_center = detect_peduncle(peduncle_img, 
                                                      distance_threshold, 
                                                      bg_img = segment_img_bright, 
                                                      save = True, 
                                                      name = tomato_name, 
                                                      pwd = pwd_results)
                        
        duration = time.time() - start_time                                            
        print("--- %.2f seconds ---" % duration)