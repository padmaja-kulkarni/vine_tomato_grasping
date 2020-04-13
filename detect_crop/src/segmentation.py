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
from detect_crop.util import save_fig
from detect_crop.util import stack_segments
from detect_crop.util import segmentation_otsu_test

#%% init
pathCurrent = os.path.dirname(__file__)
dataSet = "tomato_real"
fileName = "tomato_RGB_001.png"
imMax = 255

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", "clustering")

imPath = os.path.join(pwdData, fileName)
imBGR = cv2.imread(imPath)
[H, W] = imBGR.shape[:2]

# Cropping, only works for this specific image!
h = int(H/2)
w = int(W/2)
row = int(H/4)
col = int(w/1.5)
imBGR = imBGR[row:row + h, col:col + w]

plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

# color spaces
imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
imYCrCb = cv2.cvtColor(imRGB, cv2.COLOR_RGB2YCrCb)

#%%######################
### background, truss ###
#########################

# RGB
# background, tomato, peduncle = segmentation_otsu_test(imRGB[:,:,1], imHSV[:,:,0], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
# save_fig(segmentsRGB, pwdResults, '1RGB', figureTitle = 'Green (RGB)')

# HSV
background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imHSV[:,:,0], imMax)
segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
save_fig(segmentsRGB, pwdResults, '1HSV', figureTitle = 'Saturation (HSV)')

# LAB
# background, tomato, peduncle = segmentation_otsu_test(imLAB[:,:,1], imHSV[:,:,0], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, np.zeros(tomato.shape, dtype = np.uint8))
# save_fig(segmentsRGB, pwdResults, '1LAB', figureTitle = 'A (LAB)')


#%%#####################
### tomato, peduncle ###
########################
# # HSV
# background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imRGB[:,:,0], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
# save_fig(segmentsRGB, pwdResults, '2RGB', figureTitle = 'R (RGB)')


# HUE
# background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imHSV[:,:,0], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
# save_fig(segmentsRGB, pwdResults, '2HSV', figureTitle = 'Hue (HSV)')

# background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imLAB[:,:,1], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
# save_fig(segmentsRGB, pwdResults, '2LAB', figureTitle = 'A (LAB)')



# background, tomato, peduncle = segmentation_otsu_test(imHSV[:,:,1], imYCrCb[:,:,1], imMax)
# segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
# save_fig(segmentsRGB, pwdResults, '2YCrCb', figureTitle = 'Cr (YCbCr)')