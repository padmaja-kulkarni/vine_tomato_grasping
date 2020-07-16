# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:16:42 2020

@author: taeke
"""

import os
from detect_crop.util import make_dirs
from detect_crop.util import change_brightness

import time

from detect_crop.ProcessImage import ProcessImage 
from detect_crop.detect_peduncle import detect_peduncle
import cv2

if __name__ == '__main__':
    
    #%% init
     #  48 #        # tomato file to load
    nDigits = 3
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "empty" # "artificial" # "real_blue" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_peduncle")
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255

    brightness = 0.85
    
    for iTomato in range(1,2):
        
        tomatoID = str(iTomato).zfill(nDigits)
        tomatoName = tomatoID # "tomato" + "_RGB_" + 
        fileName = tomatoName + ".png" # ".jpg" # 
        
        
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
        image.detect_tomatoes()
    
        distance_threshold = 10
        tomatoes = image.get_tomatoes(local = True)
        
        segment_img = image.get_segmented_image(local = True)   
        peduncle_img = image.get_peduncle_image()
        segment_img_bright = change_brightness(segment_img, brightness)
              

        start_time = time.time()
        detect_peduncle(peduncle_img, distance_threshold, save = True, 
                        bg_img = segment_img_bright, name = tomatoName, 
                        pwd = pwdResults)
                        
        print("--- %.2f seconds ---" % (time.time() - start_time))