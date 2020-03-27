#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 17:05:38 2020

@author: taeke
"""

## imports ##
import pathlib
import os # os.sep
import cv2
import numpy as np

from matplotlib import pyplot as plt

# custom functions
from image_processing import save_fig
from image_processing import stack_segments
from image_processing import segmentation_otsu

from image_processing import load_rgb
from image_processing import romove_blobs
from image_processing import save_fig

# prior knowledge
filterDiameterTom = 11
filterDiameterPend = 5

pathCurrent = pathlib.Path().absolute()
dataSet = "tomato_rot"
imMax = 255

iTomato = 2
nDigits = 3

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", "morphological")


tomatoName = "tomato" + "_RGB_" + str(iTomato).zfill(nDigits) 
fileName = tomatoName + ".png" # png
    
imRGB, DIM = load_rgb(pwdData, fileName, horizontal = True)


# Cropping, only works for this specific image!
h = int(DIM[0]/2)
w = int(DIM[1]/2)
row = DIM[0] - h
col = int(w/1.5)
imRGB = imRGB[row:row + h, col:col + w]

plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20



background, tomato, peduncle = segmentation_otsu(imRGB, imMax)

segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
save_fig(segmentsRGB, pwdResults, '1')


#%%###########
### Filter ###
##############

# tomato
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterTom, filterDiameterTom))
tomatoFiltered = cv2.morphologyEx(cv2.morphologyEx(tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)

# peduncle
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterPend, filterDiameterPend))
peduncleFiltered = cv2.morphologyEx(cv2.morphologyEx(peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
peduncleFiltered = romove_blobs(peduncleFiltered, imMax)

# background
backgroundFiltered = cv2.bitwise_not(tomatoFiltered)
 
segmentsRGB = stack_segments(imRGB, backgroundFiltered, tomatoFiltered, peduncleFiltered)
save_fig(segmentsRGB, pwdResults, '2')