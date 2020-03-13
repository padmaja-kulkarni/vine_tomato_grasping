#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 21:21:28 2020

@author: taeke
"""

import os
import cv2
import numpy as np
import warnings

from matplotlib import pyplot as plt

def rgb2hsi(RGB):
    
    RGB = RGB.astype('int64')
    
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


    #Hue
    #numi =1/2*((R - G) + (R - B))
    #denom=np.sqrt( np.power(R - G,2) + (R - B)*(G - B))

    #To avoid divide by zero exception add a small number in the denominator
    #H= np.arccos(numi/(denom + 0.0001))

    #If B>G then H= 360-Theta
    #H = (G >= B) * H + (G < B) * (2*np.pi - H)

    #Normalize to the range [0 1]
    #H=H/(2*np.pi)

    #Saturation
   
    #Intensity
    I=(R + G + B)/3

    S= 1 - np.amin(RGB, 2) /np.sum(RGB, 2)


    H = H/2
    S = S * 255
    HSI = np.dstack((np.dstack((H, S)), I))
    return HSI

def bin2img(binary):
    img = binary.astype(np.uint8) * 255
    return img

def romove_blobs(imgIn, imMax):
    [H, W] = imgIn.shape[:2]
    
    # the extra [-2:] is essential to make it work with varios cv2 ersions
    contours, _ = cv2.findContours(imgIn, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    cnt = max(contours, key=cv2.contourArea)
    
    imgOut = np.zeros((H,W), np.uint8)
    cv2.drawContours(imgOut, [cnt], -1, imMax, cv2.FILLED)
    imgOut = cv2.bitwise_and(imgIn, imgOut)
    return imgOut


def romove_blobs_2(imgIn, imMax):
    [H, W] = imgIn.shape[:2]
    
    # the extra [-2:] is essential to make it work with varios cv2 ersions
    contours, _ = cv2.findContours(imgIn, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    cnt = max(contours, key=cv2.contourArea)
    
    imgOut = np.zeros((H,W), np.uint8)
    cv2.drawContours(imgOut, [cnt], -1, imMax, cv2.FILLED)
    return imgOut

def add_border(imOriginal, location, sizeBorder):

    sizeOriginal = imOriginal.shape
    location = location.astype(int)
    
    imBorder = np.zeros(sizeBorder[0:2], np.uint8)

    colStart = location[0, 0]
    colEnd = location[0, 0] + sizeOriginal[1]
    
    rowStart = location[0, 1]
    rowEnd = location[0, 1] + sizeOriginal[0]
    
    
    
    if (rowEnd > sizeBorder[0]):
        warnings.warn('cutting immage')
        rowEnd = sizeBorder[0]
    
   
    imBorder[rowStart:rowEnd, colStart:colEnd] = imOriginal[0:rowEnd - rowStart, :]
    
    return imBorder


def segmentation_parametric(imRGB, imMax):
    
    # hsi segmentation
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    # plt.figure(), plt.imshow(imHSV)[:,:, 1] 
    
    a, background = cv2.threshold(imHSV[:,:,1], 0.4*imMax, imMax, cv2.THRESH_BINARY_INV)
    tomato = cv2.bitwise_not(background)
    # a, peduncle = cv2.threshold(imHSV[:,:,0], 0.1 * imMax, imMax, cv2.THRESH_BINARY_INV)
    peduncle = (imHSV[:,:,0] > 0.1*imMax) * tomato
    
    return background, tomato, peduncle


def segmentation_cluster(imRGB, imMax):
    
    # init
    [H, W] = imRGB.shape[:2]
    imHSI = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    
    imS = imHSI[:,:,1]
    data = np.float32(imS.reshape((H * W), 1))

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(data,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    # compute masks
    tomato = label.ravel() == np.argmax(center)
    tomato = tomato.reshape((H, W))
    tomato = bin2img(tomato) #convert to an unsigned byte
    tomato = romove_blobs_2(tomato, imMax)
    background = cv2.bitwise_not(tomato)
    

    
    # seperate tomato from peduncle
    imH = imHSI[:,:,0]
    data = np.float32(imH.reshape((H * W), 1))   
    dataCut = data[(tomato == 255).flatten()]
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(dataCut,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    
    # label
    peduncle = np.logical_and(tomato, imH > np.mean(center))
    peduncle = bin2img(peduncle)
    peduncle = romove_blobs(peduncle, imMax)
    
    
    # pixelLabel[peduncle == 255] = 2
    
    
    # background = bin2img(pixelLabel == 1)
    # tomato = bin2img(np.logical_or(pixelLabel == 0, pixelLabel == 2))
    # peduncle = bin2img(pixelLabel == 2)
    
    return background, tomato, peduncle

def rot2or(loc, dim, alpha):
    # loc in [rows, cols]
    
    N = loc.shape[0]
    LOC = np.empty((N,2))
    
    for i in range(0, N):
        col = loc[i, 0]
        row = loc[i, 1]
        H = dim[0]
        W = dim[1]
        
        if (alpha > np.pi or alpha < -np.pi):
            warnings.warn('Are you using radians?')
        
        # trig equations depend on angle
        if alpha < 0:
            COL = col*np.cos(alpha) - row*np.sin(alpha) + np.cos(alpha)*np.sin(alpha)*H
            ROW = col*np.sin(alpha) + row*np.cos(alpha) + np.sin(alpha)*np.sin(alpha)*H
        else:
            COL = col*np.cos(alpha) - row*np.sin(alpha) + np.sin(alpha)*np.sin(alpha)*W
            ROW = col*np.sin(alpha) + row*np.cos(alpha) - np.cos(alpha)*np.sin(alpha)*W
            
        LOC[i, :] = np.matrix((COL, ROW))
    return LOC

def or2rot(loc, dim, alpha):

    if (alpha > np.pi or alpha < -np.pi):
         warnings.warn('Are you using radians?')

    x = loc[0, 0]
    y = loc[0, 1]
    H = dim[0]
    W = dim[1]

    if alpha > 0:
        X = 1
        Y = W * np.sin(alpha)
    else:
        X = -H * np.sin(alpha)
        Y = 1
    
    LOC = np.matrix((X, Y))
    return LOC

def segmentation_cluster_test(im1, im2, imMax):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    # init
    [h, w] = im1.shape[:2]
    data1 = np.float32(im1.reshape((h * w), 1))
    data2 = np.float32(im2.reshape((h * w), 1))   

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(data1,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    # compute masks
    tomato = label.ravel() == np.argmax(center)
    tomato = tomato.reshape((h, w))
    tomato = bin2img(tomato) #convert to an unsigned byte
    #tomato = romove_blobs_2(tomato, imMax)
    background = cv2.bitwise_not(tomato)
    
    # seperate tomato from peduncle
    dataCut = data2[(tomato == 255).flatten()]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(dataCut,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    # label
    peduncle = np.logical_and(tomato, im2 > np.mean(center))
    peduncle = bin2img(peduncle)
    peduncle = romove_blobs(peduncle, imMax)

    return background, tomato, peduncle

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

def segmentation_otsu_test(im1, im2, imMax):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
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

# used to be called visualize_cluster
def stack_segments(imRGB, background, tomato, peduncle):
    # stack segments
    
    [h, w] = imRGB.shape[:2]
    
    # set labels
    backgroundLabel = 0
    tomatoLabel = 1
    peduncleLabel = 2
    
    # label pixels
    pixelLabel = np.zeros((h,w), dtype= np.int8)
    pixelLabel[background > 0] = backgroundLabel
    pixelLabel[cv2.bitwise_and(tomato, cv2.bitwise_not(peduncle))>0] = tomatoLabel
    pixelLabel[peduncle > 0] = peduncleLabel
    
    # get class colors
    colorBackground  = np.uint8(np.mean(imRGB[pixelLabel == backgroundLabel], 0))
    colorTomato = np.uint8(np.mean(imRGB[pixelLabel == tomatoLabel], 0))
    colorPeduncle = np.uint8(np.mean(imRGB[pixelLabel == peduncleLabel], 0))
    color = np.vstack([colorBackground, colorTomato, colorPeduncle])
    
    # visualize
    res = color[pixelLabel.flatten()]
    res2 = res.reshape((h,w,3))
    
    
    return res2

def save_fig(img, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, saveFormat = 'pdf'):
        plt.rcParams["savefig.format"] = saveFormat
        plt.rcParams["savefig.bbox"] = 'tight' 
        plt.rcParams['axes.titlesize'] = titleSize
    
        fig = plt.figure() 
        plt.imshow(img)
        plt.title(figureTitle)
        plt.axis('off')
        fig.savefig(os.path.join(pwd, name), dpi = resolution, pad_inches=0)
        
def load_rgb(pwd, name, horizontal = True):
    
    #load image
    imPath = os.path.join(pwd, name)
    imBGR = cv2.imread(imPath)
    imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
    # plt.imshow(imRGB) 
    
    if horizontal:
        [H, W] = imRGB.shape[:2]
        if H > W :
            imRGB = np.transpose(imRGB, [1,0,2])
            [H, W] = imRGB.shape[:2]
    
    [H, W] = imRGB.shape[:2]
    DIM = [H,W]
    
    return imRGB, DIM

def plot_circles(imRGB, centers, radii, savePath = None, saveName = None, figureTitle = "", titleSize = 20, resolution = 300):
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = titleSize
    
    fig = plt.figure() 
    plt.subplot(1, 1, 1)
    ax = fig.gca()
    
    if radii is not None:
        for i in range(0, len(radii), 1):
            # draw the outer circle
            circle = plt.Circle((centers[i, 0], centers[i, 1]), radii[i], color='r', fill=False, lw=5)
            ax.add_artist(circle)
    
    plt.imshow(imRGB)
    plt.title(figureTitle)
    plt.axis('off')
    
    if savePath is not None:
        fig.savefig(os.path.join(savePath, saveName), dpi = resolution, pad_inches=0)
    return fig