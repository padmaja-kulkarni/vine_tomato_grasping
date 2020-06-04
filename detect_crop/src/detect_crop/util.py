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


def make_dirs(pwd):
    if not os.path.isdir(pwd):
        print("New path, creating a new folder: " + pwd)
        os.makedirs(pwd)

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
    
    if contours == []:
        return imgIn

    else:
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


def segmentation_truss_sim(img_saturation, img_hue, img_A, imMax):
    
        im0 = img_saturation    # saturation
        im1 = img_hue           # hue
        im2 = img_A             # A
 
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
        
        return background, tomato, peduncle
        
def segmentation_truss_real_simpel(img_hue, img_A, imMax):

        im1 = img_hue # hue
        im2 = img_A # A

        # Seperate truss from background
        data1 = im1.flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        thresholdTruss = 15
        temp, truss_1 = cv2.threshold(im1,thresholdTruss,imMax,cv2.THRESH_BINARY_INV)
        temp, truss_2 = cv2.threshold(im1,thresholdTruss + 120,imMax,cv2.THRESH_BINARY) # circle
        tomato = cv2.bitwise_or(truss_1,truss_2)
        
        # seperate tomato from peduncle
        # data2 = im2[(truss == imMax)].flatten()
        # threshPeduncle, temp = cv2.threshold(data2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
   
        temp, peduncle = cv2.threshold(im1,60,imMax,cv2.THRESH_BINARY)
        #temp, peduncle_2 = cv2.threshold(im1,thresholdTruss,imMax,cv2.THRESH_BINARY)
        # peduncle = cv2.bitwise_and(peduncle_1, peduncle_2)
        truss = cv2.bitwise_or(tomato, peduncle) # 
        background = cv2.bitwise_not(truss)    
        
def segmentation_truss_real(img_hue, imMax):
        im1 = img_hue # hue
 
        # Seperate truss from background
        data1 = im1.flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, truss_1 = cv2.threshold(im1,15,imMax,cv2.THRESH_BINARY_INV)
        temp, truss_2 = cv2.threshold(im1,150,imMax,cv2.THRESH_BINARY) # circle
        truss = cv2.bitwise_or(truss_1,truss_2)
        background = cv2.bitwise_not(truss)
        
        # seperate tomato from peduncle
        data2 = im1[(truss == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(data2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
   

        temp, peduncle_1 = cv2.threshold(im1,60,imMax,cv2.THRESH_BINARY_INV)
        temp, peduncle_2 = cv2.threshold(im1,15,imMax,cv2.THRESH_BINARY)
        peduncle = cv2.bitwise_and(peduncle_1, peduncle_2)
        
        return background, truss, peduncle
        
def segmentation_tomato_real(img_a, imMax):
        im1 = img_a # hue
 
        # Seperate truss from background
        data1 = im1.flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        

        temp, truss = cv2.threshold(im1,thresholdTomato,imMax,cv2.THRESH_BINARY_INV)
        background = cv2.bitwise_not(truss)
        
        peduncle = np.zeros(truss.shape, dtype = np.uint8)
        
        return background, truss, peduncle

def segmentation_blue(imRGB, imMax):
        imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
        imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
        
        #%%#################
        ### SEGMENTATION ###
        ####################
    
        im1 = imHSV[:, :, 1]
        im2 = imLAB[:, :, 1]
        

        # Otsu's thresholding
        thresholdTomato, temp = cv2.threshold(im2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, tomato = cv2.threshold(im2,thresholdTomato,imMax,cv2.THRESH_BINARY)
        background = cv2.bitwise_not(tomato)
        

        # seperate tomato from peduncle
        dataCut = im1[(background == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#        
#        # label
        temp, peduncle = cv2.threshold(im1,threshPeduncle,imMax,cv2.THRESH_BINARY)
        peduncle = cv2.bitwise_and(peduncle, background)
        
        return background, tomato, peduncle

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
    threshholdTruss,tomato = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    background = cv2.bitwise_not(tomato)
    
    # seperate tomato from peduncle
    dataCut = data2[(tomato == imMax).flatten()]
    threshPeduncle,peduncle = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
    peduncle = cv2.bitwise_and(tomato, peduncle)

    return background, tomato, peduncle

def segmentation_cluster_test(im1, im2, imMax, pwd, name):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    # init
    [h, w] = im1.shape[:2]
    data1 = np.float32(im1.reshape((h * w), 1))
    data2 = np.float32(im2.reshape((h * w), 1))   

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(data = data1,
                                K = 2,
                                bestLabels = None,
                                criteria = criteria,
                                attempts = 10,
                                flags = cv2.KMEANS_RANDOM_CENTERS)
    
    # compute masks
    truss = label.ravel() == np.argmax(center)
    truss = truss.reshape((h, w))
    truss = bin2img(truss) 
    
    background_1 = cv2.bitwise_not(truss)
    
    #%%#################
    ### SECOND STAGE ###
    ####################
    K = 3
    labelSet = set(np.arange(0,K))
    dataCut = data2[(truss == 255).flatten()]
    centersInit = np.linspace(start = [0], stop = [256], num = K) # np.array([[0], [256/2], [256]])
    labelsInit = label_img(dataCut, centersInit)
    
    
    _, _, centers = cv2.kmeans(data = dataCut,
                                   K = K,
                                   bestLabels = labelsInit,
                                   criteria = criteria,
                                   attempts = 1,
                                   flags = cv2.KMEANS_USE_INITIAL_LABELS)
   
    # determine which center corresponds to which label
    peduncleLabel = np.argmin(centers)
    tomatoLabel = np.argmax(centers)
    
    if K == 3:
        backgroundLabel = list(labelSet - set( [peduncleLabel, tomatoLabel]))[0]   
    else:
        backgroundLabel = None
    # apply to entire image
    labels = label_img(data2, centers)
    
    tomato = label2img(labels, tomatoLabel, h ,w) 
    tomato = cv2.bitwise_and(tomato, truss)
    
    peduncle = label2img(labels, peduncleLabel, h ,w)
    peduncle = cv2.bitwise_and(peduncle, truss)
    
    if backgroundLabel is not None:
        background_2 = label2img(labels, backgroundLabel, h ,w)
        background = cv2.bitwise_or(background_1, background_2)
        truss = cv2.bitwise_not(background)
    else:
        background = background_1
    
    fig, ax= plt.subplots(1)
    ax.set_title('A (LAB)')
    ax.hist(dataCut.ravel(), bins=256, range=(0,255))
    ax.set_xlim(0, 255)
    ax.axvline(x=centers[peduncleLabel],  color='g')
    ax.axvline(x=centers[tomatoLabel],  color='r')
    
    if backgroundLabel is not None:
        ax.axvline(x=centers[backgroundLabel],  color='k')
    save_fig(fig, pwd, name + "_hist_2_k_means", figureTitle = "", resolution = 100, titleSize = 10)
    

    return background, tomato, peduncle,truss


def label2img(labels, label, h ,w):
    data = labels.ravel() == label
    img = data.reshape((h, w))
    return bin2img(img)   

def label_img(data, centers):
    dist = abs(data - np.transpose(centers))
    labels = np.argmin(dist,1).astype(np.int32)
    return np.expand_dims(labels, axis=1)    

def segmentation_otsu_test(im1, im2, imMax, pwd, name):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    
  
    # init
    [h, w] = im1.shape[:2]
    data2 = im2.reshape((h * w), 1) 
    
    # Otsu's thresholding
    thresholdTruss, temp = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    temp, truss = cv2.threshold(im1,thresholdTruss,imMax,cv2.THRESH_BINARY)
    background_1 = cv2.bitwise_not(truss)
    
    # seperate tomato from peduncle
    dataCut = data2[(truss == imMax).flatten()]
    threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle_1 = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY_INV)
    temp, tomato = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
    peduncle_1 = cv2.bitwise_and(truss, peduncle_1)

    dataCut2 = data2[(peduncle_1 == imMax).flatten()]
    threshPeduncle_2, temp = cv2.threshold(dataCut2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY_INV)
    temp, background_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY)
    
    peduncle = cv2.bitwise_and(peduncle_2, peduncle_1)
    background = cv2.bitwise_or(background_2, background_1)
    # tomato = cv2.bitwise_and(cv2.bitwise_not(peduncle), cv2.bitwise_not(background))

    fig, ax= plt.subplots(1)
#    fig.suptitle('Histogram')
    
    ax.set_title('S (HSV)')
    values = ax.hist(im1.ravel(), bins=256/1, range=(0, 255))
    ax.set_ylim(0, 4*np.mean(values[0]))
    ax.set_xlim(0, 255)
    ax.axvline(x=thresholdTruss,  color='r')
    save_fig(fig, pwd, name + "_hist_1", figureTitle = "", resolution = 100, titleSize = 10)

    fig, ax= plt.subplots(1)
    ax.set_title('A (LAB)')
    values = ax.hist(dataCut.ravel(), bins=256/1, range=(0,255))
    ax.set_ylim(0, 4*np.mean(values[0]))
    ax.set_xlim(0, 255)
    ax.axvline(x=threshPeduncle,  color='r')
    ax.axvline(x=threshPeduncle_2,  color='r')
    save_fig(fig, pwd, name + "_hist_2", figureTitle = "", resolution = 100, titleSize = 10)

    return background, tomato, peduncle, truss

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

def save_img(img, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, saveFormat = 'png'):
        plt.rcParams["savefig.format"] = saveFormat
        plt.rcParams["savefig.bbox"] = 'tight' 
        plt.rcParams['axes.titlesize'] = titleSize
        
    
        fig = plt.figure() 
        plt.imshow(img)
        plt.axis('off')
        plt.title(figureTitle)
        
        # https://stackoverflow.com/a/27227718
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
            hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        
        
        fig.savefig(os.path.join(pwd, name), dpi = resolution, bbox_inches='tight', pad_inches=0)
        
def save_fig(fig, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, saveFormat = 'png'):
        
        SMALL_SIZE = 8
        MEDIUM_SIZE = 15
        BIGGER_SIZE = 20
    
        plt.rcParams["savefig.format"] = saveFormat
        plt.rcParams["savefig.bbox"] = 'tight' 
        # plt.rcParams['axes.titlesize'] = titleSize
        
        plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
    
       
        for ax in fig.get_axes():
            ax.label_outer()
        
        
        for ax in fig.get_axes():
            # ax.yaxis.set_major_locator(plt.nulllocator())\
            ax.set_yticklabels([])
        plt.margins(0,0) 
        
        fig.savefig(os.path.join(pwd, name), dpi = resolution, bbox_inches='tight', pad_inches=0)
        
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


def add_circles(imRGB, centers, radii):
    if radii is not None:
        for i in range(0, len(radii), 1):
            col = int(centers[i, 0])
            row = int(centers[i, 1])
            r = int(radii[i])
            cv2.circle(imRGB,(col, row), r, (0,255,0))
            
    return imRGB

def plot_circles(imRGB, centers, radii, savePath = None, saveName = None, 
                 figureTitle = "", titleSize = 20, resolution = 300, 
                 fileFormat = 'pdf'):
    
    plt.rcParams["savefig.format"] = fileFormat 
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