#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 14:35:14 2020

@author: taeke
"""


import numpy as np
from matplotlib import pyplot as plt

from skimage.transform import rotate

# custom
from detect_crop.util import rot2or
from detect_crop.util import or2rot

import cv2

#%% INIT
H = 40
W = 100

row = 10
col = 40

pixelO = (row, col) # row, col

# for angle_deg in range(0, 90, 10):

for angle_deg in range(-90, 90, 10):
    angle_rad = angle_deg/180.0*np.pi
    
    
    # create image
    DIM = (H,W)
    img = np.ones(DIM)
    img[pixelO] = 10
    
    # rotate image
    imgR = rotate(img, angle_deg, resize=True)    
    
    # detect new pixel location
    pixel_r = np.unravel_index(np.argmax(imgR, axis=None), imgR.shape)
    
    pixel_r = np.array([[pixel_r[0]], [pixel_r[1]]])
    R = np.array([[np.cos(angle_rad), np.sin(angle_rad)],[ -np.sin(angle_rad), np.cos(angle_rad)]])
    
    if angle_deg > 0:
        T = np.array([[-np.sin(angle_rad) * W], [0]])
    else:
        T = np.array([[0], [np.sin(angle_rad) * H]])
            
    pixel_p = np.matmul(R, (pixel_r + T))
    
    # pixelR = np.matrix((pixelR[0], pixelR[1])) # row, col
    
    # rotate back
    centersO = pixel_p # rot2or(pixelR, DIM, angle/180*np.pi)
    
    
    fig = plt.figure()
    plt.subplot(2, 2, 1)
    plt.imshow(img)
    
    
    plt.subplot(2, 2, 2)
    plt.imshow(imgR)
    
    plt.subplot(2, 2, 3)
    ax = fig.gca()
    plt.imshow(img)
    
    
    circle = plt.Circle((centersO[1], centersO[0]), 3, color='r', fill=False)
    ax.add_artist(circle)