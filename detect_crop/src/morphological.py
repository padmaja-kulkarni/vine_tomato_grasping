#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 17:05:38 2020

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
from detect_crop.util import segmentation_truss_real

from detect_crop.util import load_rgb
from detect_crop.util import romove_blobs

# prior knowledge
filterDiameterTom = 11
filterDiameterPend = 5

pathCurrent = os.path.dirname(__file__)
dataSet = "real_blue"
imMax = 255

nDigits = 3

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", dataSet, "morphological")

# create folder if required
if not os.path.isdir(pwdResults):
    print("New data set, creating a new folder: " + pwdResults)
    os.makedirs(pwdResults)

for iTomato in range(1, 2):

    
    tomatoName = str(iTomato).zfill(nDigits) 
    fileName = tomatoName + ".png" # png
        
    imRGB, DIM = load_rgb(pwdData, fileName, horizontal = True)
    H = DIM[0]
    W = DIM[1]
    
    # Cropping, only works for this specific image!
    h = int(H/2)
    w = int(W/2)
    
    row = int(H/6)
    col = int(W/3)
    # imRGB = imRGB[row:row + h, col:col + w]
    
    plt.rcParams["image.cmap"] = 'plasma'
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = 20
    
    
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    background, tomato, peduncle = segmentation_truss_real(imHSV[:,:,0], imMax)
    
    
    #%%###########
    ### Filter ###
    ##############
    
    # tomato
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterTom, filterDiameterTom))
    # tomatoFiltered = tomato # cv2.morphologyEx(cv2.morphologyEx(tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
    
    # remove blobs truss
    truss = cv2.bitwise_or(tomato, peduncle)
    trussFiltered = romove_blobs(truss, imMax)
    peduncleFiltered = cv2.bitwise_and(trussFiltered, peduncle)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, tomato)
    
    # peduncle
#     kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterPend, filterDiameterPend))
#     peduncleFiltered = cv2.morphologyEx(cv2.morphologyEx(peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
    
    # remove blobs peduncle
    peduncleFiltered = romove_blobs(peduncleFiltered, imMax)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, cv2.bitwise_not(peduncleFiltered))
    trussFiltered = cv2.bitwise_or(tomatoFiltered, peduncleFiltered)
    backgroundFiltered = cv2.bitwise_not(truss)
    # 
    
    # original
    segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    save_img(segmentsRGB, pwdResults, tomatoName + '_1')
    
    
    # save filtered
    segmentsRGB = stack_segments(imRGB, backgroundFiltered, tomatoFiltered, peduncleFiltered)
    save_img(segmentsRGB, pwdResults, tomatoName + '_2')