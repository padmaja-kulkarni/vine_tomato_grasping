# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 19:49:14 2020

@author: taeke
"""

import os
import cv2
import time

from detect_crop.util import change_brightness
from detect_crop.util import make_dirs
from detect_crop.ProcessImage import ProcessImage

from detect_crop.detect_tomato import detect_tomato

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
        
    
    
    for iTomato in range(1,2): 
        
        tomatoID = str(iTomato).zfill(nDigits)
        tomatoName = tomatoID
        fileName = tomatoName + ".png"
        
        
        imPath = os.path.join(pwdData, fileName)
        imBGR = cv2.imread(imPath)
        
        imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
        
        image = ProcessImage(camera_sim = False,
                                         use_truss = True,
                                         tomatoName = 'ros_tomato',
                                         pwdProcess = pwdResults,
                                         saveIntermediate = False)
        
        image.add_image(imRGB)    
        
        image.color_space()
        image.segment_truss()
        image.filter_img()
        image.rotate_cut_img()
        
        # set parameters
        blur_size = (3,3)
        radius_min = 8
        radius_max = 4
        distance_min = radius_max
        dp = 4
        param1 = 20
        param2 = 50      
        ratio_threshold = 0.5

        imageRGB = image.get_image(local = True)
        imageRGB_bright = change_brightness(imageRGB, brightness)

        image_tomato, image_peduncle, temp = image.get_segments(local = True)
        image_gray = cv2.bitwise_or(image_tomato, image_peduncle)
#        image_gray = imageRGB[:,:,0]

        start_time = time.time()
        centers, radii, com = detect_tomato(image_gray, imageRGB = imageRGB_bright, 
                      blur_size = blur_size, radius_min = radius_min, 
                      radius_max = radius_max, distance_min = distance_min,
                      dp = dp, param1 = param1, param2 = param2,
                      ratio_threshold = ratio_threshold,
                      save = True, pwd = pwdResults, name = tomatoName)

        duration = time.time() - start_time
        print("--- %.2f seconds ---" % (duration))