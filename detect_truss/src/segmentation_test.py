# -*- coding: utf-8 -*-
"""
Created on Wed May 20 13:32:31 2020

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
from detect_crop.util import make_dirs
from detect_crop.util import save_fig




#%% init

# tomato rot: 15
# tomato cases: 48
dataSet = "sim_blue"


settings = {
    "rot": {
        "extension": ".png",
        "files": 14,
        "name": "tomato_rot"},
    "cases": {
        "extension": ".jpg",
        "files": 47,
        "name": "tomato_cases"},
    "real": {
        "extension": ".png",
        "files": 1,
        "name": "tomato_real"},
    "blue": {
        "extension": ".jpg",
        "files": 1,
        "name": "tomato_blue"},
    "real_blue": {
        "extension": ".png",
        "files": 2,
        "name": "real_blue"},
    "sim_blue": {
        "extension": ".png",
        "files": 1,
        "name": "sim_blue"},
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
pwdResults = os.path.join(pathCurrent, "results", dataSet, "segmentation")

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
        im0 = imHSV[:, :, 1] # suturation
        im1 = imHSV[:, :, 0] # hue
        im2 = imLAB[:, :, 1] # A
 
        # Seperate robot from rest
        data0 = im0.flatten()
        thresholdRobot, temp = cv2.threshold(data0,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)    
        temp, robot = cv2.threshold(im0, thresholdRobot, imMax, cv2.THRESH_BINARY_INV)
        not_tobot = cv2.bitwise_not(robot)


        # Seperate truss from background
        data1 = im1[(not_tobot == imMax)].flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, truss_1 = cv2.threshold(im1,thresholdTomato,imMax,cv2.THRESH_BINARY_INV)
        temp, truss_2 = cv2.threshold(im1,thresholdTomato + 90,imMax,cv2.THRESH_BINARY) # circle
        truss = cv2.bitwise_and(cv2.bitwise_or(truss_1,truss_2), not_tobot)
        background = cv2.bitwise_or(cv2.bitwise_not(truss), robot)
        
        # seperate tomato from peduncle
        data2 = im2[(truss == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(data2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
   
        temp, tomato_temp = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
        tomato = cv2.bitwise_and(tomato_temp, truss)
        peduncle = cv2.bitwise_and(cv2.bitwise_not(tomato_temp), truss)

    
        # Plot Saturation (HSV)
        fig, ax= plt.subplots(1)
        fig.suptitle('Histogram')
        ax.set_title('Saturation (HSV)')
        
        values = ax.hist(data0, bins=256/1, range=(0, 255))
        # ax.set_ylim(0, 4*np.mean(values[0]))
        ax.set_xlim(0, 255)
        ax.axvline(x=thresholdRobot,  color='r')
        save_fig(fig, pwdResults, tomatoID + "_hist_1", figureTitle = "", resolution = 100, titleSize = 10)
    
    
        # plot Hue (HSV)
        fig, ax= plt.subplots(1)
        fig.suptitle('Histogram')
        ax.set_title('H (HSV)')
        
        values = ax.hist(data1, bins=256/1, range=(0, 255))
        # ax.set_ylim(0, 4*np.mean(values[0]))
        ax.set_xlim(0, 255)
        ax.axvline(x=thresholdTomato,  color='r')
        save_fig(fig, pwdResults, tomatoID + "_hist_2", figureTitle = "", resolution = 100, titleSize = 10)
    
    
        # plot A (LAB)
        fig, ax= plt.subplots(1)
        fig.suptitle('Histogram')
        ax.set_title('A (LAB)')
        
        values = ax.hist(data2, bins=256/1, range=(0,255))
        # ax.set_ylim(0, 4*np.mean(values[0]))
        ax.set_xlim(0, 255)
        ax.axvline(x=threshPeduncle,  color='r')
        save_fig(fig, pwdResults, tomatoID + "_hist_3", figureTitle = "", resolution = 100, titleSize = 10)
        
        segmentsRGB = stack_segments(imRGB, background, truss, np.zeros(tomato.shape, dtype = np.uint8))
    
        figureTitle = ""
        save_img(segmentsRGB, pwdResults, tomatoID + "_img_1", figureTitle = figureTitle)
        
    
        segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
        save_img(segmentsRGB, pwdResults, tomatoID + "_img_2", figureTitle = figureTitle)
    
    count = count + 1
    print("completed image %d out of %d" %(count, N))