# -*- coding: utf-8 -*-
"""
Created on Wed May 20 13:32:31 2020

@author: taeke
"""

## imports ##
import cv2
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import colors

# custom functions
from util import bin2img

def segment_truss(img_hue, imMax):

    background, tomato, peduncle, _, _, _, _ = segment_truss_extensive(img_hue, imMax)
    return background, tomato, peduncle
    
def segment_truss_extensive(img_hue, imMax):
    
    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2*np.float32(img_hue.flatten()))
    data = np.stack((np.cos(angle), np.sin(angle)), axis = 1)
 
    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 1, np.sin(np.deg2rad(5)))
    compactness,labels,centers_xy = cv2.kmeans(data, 
                                               3, 
                                               None, 
                                               criteria, 
                                               2, 
                                               cv2.KMEANS_PP_CENTERS) 

    # convert the centers from xy to angles
    centers = np.rad2deg(np.arctan2(centers_xy[:, 1], centers_xy[:, 0]))
    
    # determine which center corresponds to which segment
    label_peduncle = np.argmax(centers)
    label_background = np.argmin(centers)
    label_tomato = list(set([0, 1, 2]) - set([label_peduncle, label_background]))[0]

    # compute masks
    dim = img_hue.shape
    tomato = label2img(labels, label_tomato, dim)     
    peduncle = label2img(labels, label_peduncle, dim)   
    background = label2img(labels, label_background, dim)  
    
    return background, tomato, peduncle, label_background, label_tomato, label_peduncle, centers
        
def label2img(labels, label, dim):
    data = labels.ravel() == label
    img = data.reshape(dim)
    return bin2img(img)           
        
def segment_tomato(img_a, imMax):
        im1 = img_a # hue
 
        # Seperate truss from background
        data1 = im1.flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        

        temp, truss = cv2.threshold(im1,thresholdTomato,imMax,cv2.THRESH_BINARY_INV)
        background = cv2.bitwise_not(truss)
        
        peduncle = np.zeros(truss.shape, dtype = np.uint8)
        
        return background, truss, peduncle


def rgb2hsi(RGB):
    
    RGB = RGB.astype('float')
    
    # unsigned int!
    R, G, B = cv2.split(RGB)

    MAX = np.amax(RGB, 2) # maximum
    MIN = np.amin(RGB, 2) # minimum
    C = MAX - MIN           #
    
    rows, cols = RGB.shape[:2]
    H = np.zeros((rows,cols))
           
    # taken from https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html      
    for row in range(0, rows):
        for col in range(0, cols):
            r = R[row,col]
            g = G[row,col]
            b = B[row,col]
            if C[row,col] == 0:
                H[row,col] = 0
            elif MAX[row,col] == r:
                H[row,col] = (60*(g-b)/C[row,col]) % 360
            elif MAX[row,col] == g:
                H[row,col] = (120 + 60*(b - r)/C[row,col]) % 360
            elif MAX[row,col] == b:
                H[row,col] = (240 + 60*(r - g)/C[row,col]) % 360

    #Intensity
    I=(R + G + B)/3

    S= 1 - np.amin(RGB, 2) /np.sum(RGB, 2)

    H = H/2
    S = S * 255
    HSI = np.dstack((np.dstack((H, S)), I))
    return HSI

def hue_scatter(xy, RGB):
    
    rows, cols = RGB.shape[:2]
    
    pixel_colors = RGB.reshape((rows*cols, 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    
    fig =  plt.figure()
    ax = fig.add_subplot(2, 2, 1)
    ax.scatter(xy[:, 0], xy[:, 1], facecolors=pixel_colors, marker=".")
    ax.set_xlabel("Hue")
    ax.set_ylabel("Saturation")


    
    plt.show()