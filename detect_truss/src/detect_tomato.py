# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 19:49:14 2020

@author: taeke
"""

import os
import cv2
import time

from detect_crop.util import change_brightness
from detect_crop.util import make_dirs, load_rgb
from detect_crop.ProcessImage import ProcessImage

from detect_crop.detect_tomato import detect_tomato, set_detect_tomato_settings

if __name__ == '__main__':
    
    #%% init

    i_start = 1
    i_end = 22
    N = i_end - i_start + 1
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "tomato_cases" # "empty" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_tomato")
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255
    count = 0
    brightness = 0.85
        
    
    
    for count, i_tomato in enumerate(range(i_start, i_end + 1)):
                                                    
        print("Analyzing image %d out of %d" %(count + 1, N))        
        
        tomatoID = str(i_tomato).zfill(3)
        tomato_name = tomatoID
        file_name = tomato_name + ".png"
        
        
        
        imRGB = load_rgb(pwdData, file_name, horizontal = True)
        
        image = ProcessImage(use_truss = True)

        image.add_image(imRGB)    
        
        image.color_space()
        image.segment_image()
        image.filter_img()
        image.rotate_cut_img()
        
        # set parameters
        settings = set_detect_tomato_settings()

        img_rgb = image.get_rgb(local = True)
        img_rgb_bright = change_brightness(img_rgb, brightness)

        image_tomato, image_peduncle, _ = image.get_segments(local = True)
        image_gray = cv2.bitwise_or(image_tomato, image_peduncle)

        centers, radii, com = detect_tomato(image_gray, 
                                            settings, 
                                            img_rgb = img_rgb, 
                                            save = True, 
                                            pwd = pwdResults, 
                                            name = tomato_name)

