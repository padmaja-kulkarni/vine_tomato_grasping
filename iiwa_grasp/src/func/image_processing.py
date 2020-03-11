#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Created on Mon Mar  11 14:22:10 2020

@author: Taeke
"""

import cv2
import os
from matplotlib import pyplot as plt

def segmentation_otsu(imRGB, imMax):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    im1 = imHSV[:,:,1]
    im2 = imHSV[:,:,0]
    
    # init
    [h, w] = im1.shape[:2]
    data2 = im2.reshape((h * w), 1) 
    
    # Otsu's thresholding
    threshTomato,tomato = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    background = cv2.bitwise_not(tomato)
    
    # seperate tomato from peduncle
    dataCut = data2[(tomato == imMax).flatten()]
    threshPeduncle,peduncle = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
    peduncle = cv2.bitwise_and(tomato, peduncle)

    return background, tomato, peduncle

def save_fig(img, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, saveFormat = 'pdf'):
        plt.rcParams["savefig.format"] = saveFormat
        plt.rcParams["savefig.bbox"] = 'tight' 
        plt.rcParams['axes.titlesize'] = titleSize
    
        fig = plt.figure() 
        plt.imshow(img)
        plt.title(figureTitle)
        plt.axis('off')
        fig.savefig(os.path.join(pwd, name), dpi = resolution, pad_inches=0)