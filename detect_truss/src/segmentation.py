#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 08:52:33 2020

@author: taeke
"""


## imports ##
import os # os.sep
import cv2
import numpy as np

from matplotlib import pyplot as plt

# custom functions
from detect_crop.util import save_img
from detect_crop.util import stack_segments
from detect_crop.util import segmentation_otsu_test
from detect_crop.util import segmentation_cluster_test
from detect_crop.util import make_dirs
#%% init

# tomato rot: 15
# tomato cases: 48

def test_segmentation(dataSet, algorithm):
    settings = {
            "rot": {
                "extension": ".png",
                "files": 14,
                "name": "tomato_rot"},
            "cases": {
                "extension": ".jpg",
                "files": 47,
                "name": "tomato_cases"},
            "blue": {
                "extension": ".png",
                "files": 7,
                "name": "tomato_real_blue"},
                    }
    
    
    N = settings[dataSet]["files"]              # tomato file to load
    extension = settings[dataSet]["extension"]
    dataSet = settings[dataSet]["name"] # "tomato_rot" #  
    
    nDigits = 3
    # ls | cat -n | while read n f; do mv "$f" `printf "%03d.jpg" $n`; done
    
    
    plt.rcParams["image.cmap"] = 'plasma'
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = 20
    
    pathCurrent = os.path.dirname(__file__)
    
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "segmentation_" + algorithm)
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255
    count = 0
    
    for iTomato in range(1, N + 1):
    
        tomatoID = str(iTomato).zfill(nDigits)
        tomatoName = tomatoID # "tomato" + "_RGB_" + 
        fileName = tomatoName + extension
        
        imPath = os.path.join(pwdData, fileName)
        imBGR = cv2.imread(imPath)
        
        if imBGR is None:
            print("Failed to load image from path: %s" %(imPath))
        else:
            
            # color spaces
            imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
            imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
            imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
            
            #%%#################
            ### SEGMENTATION ###
            ####################
        
            if algorithm == "otsu":
                background, tomato, peduncle, truss = segmentation_otsu_test(imHSV[:,:,1], imLAB[:,:,1], imMax, pwdResults, tomatoID)
            elif algorithm == "kmeans":
                background, tomato, peduncle, truss = segmentation_cluster_test(imHSV[:,:,1], imLAB[:,:,1], imMax, pwdResults, tomatoID)
            
            segmentsRGB = stack_segments(imRGB, background, truss, np.zeros(tomato.shape, dtype = np.uint8))
        
            figureTitle = ""
            save_img(segmentsRGB, pwdResults, tomatoID + "_img_1", figureTitle = figureTitle)
            
        
            
            segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
            save_img(segmentsRGB, pwdResults, tomatoID + "_img_2", figureTitle = figureTitle)
        
        count = count + 1
        print("completed image %d out of %d" %(count, N))

def main():
    # test_segmentation("rot", "otsu")
    # test_segmentation("rot", "kmeans")
    # test_segmentation("cases", "otsu")
    # test_segmentation("cases", "kmeans")
    test_segmentation("blue", "kmeans")

if __name__ == '__main__':
    main()
