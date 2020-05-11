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
from detect_crop.util import make_dirs
#%% init

# tomato rot: 15
# tomato cases: 48
N = 48               # tomato file to load
nDigits = 3
# ls | cat -n | while read n f; do mv "$f" `printf "%04d.extension" $n`; done


plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

pathCurrent = os.path.dirname(__file__)
dataSet = "tomato_cases" # "tomato_rot" # 

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", dataSet, "segmentation")

make_dirs(pwdData)
make_dirs(pwdResults)

imMax = 255
count = 0

for iTomato in range(1, N):

    tomatoID = str(iTomato).zfill(nDigits)
    tomatoName = tomatoID # "tomato" + "_RGB_" + 
    fileName = tomatoName + ".jpg" #  ".png" #
    
    imPath = os.path.join(pwdData, fileName)
    imBGR = cv2.imread(imPath)
    [H, W] = imBGR.shape[:2]
    
    # Cropping, only works for this specific image!
    h = int(H/2)
    w = int(W/2)
    #row = int(H/4)
    #col = int(w/1.5)
    
    row = H - h
    col = int(w/1.5)
    # imBGR = imBGR[row:row + h, col:col + w]
    
    
    # color spaces
    imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
    # imYCrCb = cv2.cvtColor(imRGB, cv2.COLOR_RGB2YCrCb)
    
    #%%######################
    ### background, truss ###
    #########################
    # 
    
    # RGB
    # background, tomato, peduncle = segmentation_otsu_test(imRGB[:,:,1], imHSV[:,:,0], imMax)
    # segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
    # save_img(segmentsRGB, pwdResults, '1RGB', figureTitle = 'Green (RGB)')
    
    # HSV
    background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imLAB[:,:,1], imMax, pwdResults, tomatoID)
    segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
    

    figureTitle = ""
    save_img(segmentsRGB, pwdResults, tomatoID + "_img_1", figureTitle = figureTitle)
    

    
    segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    save_img(segmentsRGB, pwdResults, tomatoID + "_img_2", figureTitle = figureTitle)
    
    
    count = count + 1
    # LAB
    # background, tomato, peduncle = segmentation_otsu_test(imLAB[:,:,1], imHSV[:,:,0], imMax)
    # segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
    # save_img(segmentsRGB, pwdResults, '1LAB', figureTitle = 'A (LAB)')
    
    
    #%%#####################
    ### tomato, peduncle ###
    ########################
    # # HSV
    # background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imRGB[:,:,0], imMax)
    # segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    # save_img(segmentsRGB, pwdResults, '2RGB', figureTitle = 'R (RGB)')
    
    
    # HUE
    
    
    
    # background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imLAB[:,:,1], imMax)
    # segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    # save_img(segmentsRGB, pwdResults, '2LAB', figureTitle = 'A (LAB)')
    
    
    
    # background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imYCrCb[:,:,1], imMax)
    # segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    # save_img(segmentsRGB, pwdResults, '2YCrCb', figureTitle = 'Cr (YCbCr)')