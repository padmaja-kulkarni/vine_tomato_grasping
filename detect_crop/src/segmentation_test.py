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
from detect_crop.util import segmentation_otsu_test
from detect_crop.util import segmentation_cluster_test
from detect_crop.util import make_dirs
from detect_crop.util import save_fig
from detect_crop.util import romove_blobs
#%% init

# tomato rot: 15
# tomato cases: 48
dataSet = "real_blue"


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
        "files": 10,
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
pwdResults = os.path.join(pathCurrent, "results", dataSet, "segmentation")

make_dirs(pwdData)
make_dirs(pwdResults)

imMax = 255
count = 0

for iTomato in range(8, N + 1):

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
    
        im1 = imHSV[:, :, 0]
        im2 = imLAB[:, :, 1]
        

        # Otsu's thresholding
        thresholdTomato, temp = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, truss = cv2.threshold(im1,thresholdTomato,imMax,cv2.THRESH_BINARY_INV)
        background = cv2.bitwise_not(truss)
        

        # seperate tomato from peduncle
        dataCut = im2[(truss == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#        
#        # label
        temp, tomato_temp = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
        tomato = cv2.bitwise_and(tomato_temp, truss)
        peduncle = cv2.bitwise_and(cv2.bitwise_not(tomato_temp), truss)
#        truss = romove_blobs(cv2.bitwise_or(peduncle, tomato), imMax)        
#        
#
#        peduncle = cv2.bitwise_and(truss, cv2.bitwise_not(tomato))
#        plt.figure()
#        plt.imshow(peduncle)
#        
#        
#        dataCut = im2[(peduncle == imMax)].flatten()
#        threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)        
#        
#        # label
#        temp, peduncle_2 = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY_INV)        
#        
#        peduncle = cv2.bitwise_and(peduncle, peduncle_2)
#        temp, tomato = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
#        peduncle_1 = cv2.bitwise_and(truss, peduncle_1)
#    
#        dataCut2 = data2[(peduncle_1 == imMax).flatten()]
#        threshPeduncle_2, temp = cv2.threshold(dataCut2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#        
#        # label
#        temp, peduncle_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY_INV)
#        temp, background_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY)
#        
#        peduncle = cv2.bitwise_and(peduncle_2, peduncle_1)
#        background = cv2.bitwise_or(background_2, background_1)
#        # tomato = cv2.bitwise_and(cv2.bitwise_not(peduncle), cv2.bitwise_not(background))
#    
        fig, ax= plt.subplots(1)
        fig.suptitle('Histogram')
        
        name = tomatoID
        
        ax.set_title('S (HSV)')
        values = ax.hist(im1.ravel(), bins=256/1, range=(0, 255))
        # ax.set_ylim(0, 4*np.mean(values[0]))
        ax.set_xlim(0, 255)
        ax.axvline(x=thresholdTomato,  color='r')
        save_fig(fig, pwdResults, name + "_hist_1", figureTitle = "", resolution = 100, titleSize = 10)
    
        fig, ax= plt.subplots(1)
        ax.set_title('A (LAB)')
        values = ax.hist(im2.ravel(), bins=256/1, range=(0,255))
        # ax.set_ylim(0, 4*np.mean(values[0]))
        ax.set_xlim(0, 255)
        ax.axvline(x=thresholdTomato,  color='r')
        save_fig(fig, pwdResults, name + "_hist_2", figureTitle = "", resolution = 100, titleSize = 10)
        
        segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
    
        figureTitle = ""
        save_img(segmentsRGB, pwdResults, tomatoID + "_img_1", figureTitle = figureTitle)
        
    
#        segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
#        save_img(segmentsRGB, pwdResults, tomatoID + "_img_2", figureTitle = figureTitle)
    
    count = count + 1
    print("completed image %d out of %d" %(count, N))