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

    nDigits = 3
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "tomato_cases" # "empty" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_tomato")
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255
    count = 0
    brightness = 0.85
        
    
    
    for count, i_tomato in enumerate(range(1, N)):
        
        tomatoID = str(i_tomato).zfill(nDigits)
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

        imageRGB = image.get_rgb(local = True)
        imageRGB_bright = change_brightness(imageRGB, brightness)

        image_tomato, image_peduncle, _ = image.get_segments(local = True)
        image_gray = cv2.bitwise_or(image_tomato, image_peduncle)
#        image_gray = imageRGB[:,:,0]

        centers, radii, com = detect_tomato(image_gray, 
                                            settings, 
                                            imageRGB = imageRGB_bright, 
                                            save = True, 
                                            pwd = pwdResults, 
                                            name = tomato_name)
                                            
        print("completed image %d out of %d" %(count, N))
