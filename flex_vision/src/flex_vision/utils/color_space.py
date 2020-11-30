# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 15:40:39 2020

@author: taeke
"""

import numpy as np
import cv2


def rgb2hsi(RGB):
    
    RGB = RGB.astype('float')
    
    # unsigned int!
    R, G, B = cv2.split(RGB)

    MAX = np.amax(RGB, 2) # maximum
    MIN = np.amin(RGB, 2) # minimum
    C = MAX - MIN           #
    
    rows, cols = RGB.shape[:2]
    H = np.zeros((rows, cols))
           
    # taken from https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html      
    for row in range(0, rows):
        for col in range(0, cols):
            r = R[row, col]
            g = G[row, col]
            b = B[row, col]
            if C[row, col] == 0:
                H[row, col] = 0
            elif MAX[row, col] == r:
                H[row, col] = (60*(g-b)/C[row, col]) % 360
            elif MAX[row, col] == g:
                H[row, col] = (120 + 60*(b - r)/C[row, col]) % 360
            elif MAX[row, col] == b:
                H[row, col] = (240 + 60*(r - g)/C[row, col]) % 360


    I = (R + G + B)/3
    S = 1 - np.amin(RGB, 2) / np.sum(RGB, 2)

    H = H/2
    S = S * 255
    HSI = np.dstack((np.dstack((H, S)), I))
    return HSI