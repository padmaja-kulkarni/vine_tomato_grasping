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
        imgOut = np.zeros((H,W), np.uint8)        
        
        cnt = max(contours, key=cv2.contourArea)
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
    
def remove_all_blobs(tomato, peduncle, background, imMax):
    
    # remove blobs truss
    truss = cv2.bitwise_or(tomato, peduncle)
    trussFiltered = romove_blobs(truss, imMax)
    peduncleFiltered = cv2.bitwise_and(trussFiltered, peduncle)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, tomato)
    
    # remove blobs peduncle
    peduncleFiltered = romove_blobs(peduncleFiltered, imMax)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, cv2.bitwise_not(peduncleFiltered))
    trussFiltered = cv2.bitwise_or(tomatoFiltered, peduncleFiltered)
    backgroundFiltered = cv2.bitwise_not(truss)    
    
    
    
    return tomatoFiltered, peduncleFiltered, backgroundFiltered

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

def translation_rot2or(dim, alpha):
    
    H = dim[0]
    W = dim[1]
    
    if (alpha > np.pi or alpha < -np.pi):
        warnings.warn('Are you using radians?')
    
    # trig equations depend on angle
    if alpha < 0:
        col = np.cos(alpha)*np.sin(alpha)*H
        row = np.sin(alpha)*np.sin(alpha)*H
    else:
        col = np.sin(alpha)*np.sin(alpha)*W
        row = np.cos(alpha)*np.sin(alpha)*W
            

    return (col, row)

def or2rot(dim, alpha):

    if (alpha > np.pi or alpha < -np.pi):
         warnings.warn('Are you using radians?')

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

def label_img(data, centers):
    dist = abs(data - np.transpose(centers))
    labels = np.argmin(dist,1).astype(np.int32)
    return np.expand_dims(labels, axis=1)    


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
    
    # print('color tomato:', colorTomato)
    # print('color tomato:', colorPeduncle)
    return res2

def save_img(img, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, ext = 'png'):
        plt.rcParams["savefig.format"] = ext
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
        
def save_fig(fig, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, ext = 'png'):
        
        SMALL_SIZE = 8
        MEDIUM_SIZE = 15
        BIGGER_SIZE = 20
    
        plt.rcParams["savefig.format"] = ext
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


def add_circles(imRGB, centers, radii = 5, color = (255,255,255), thickness = 5):
    if radii is not None:
        
        if isinstance(centers, list):  
            for i, center in enumerate(centers):
                col = int(center[0])
                row = int(center[1])
            
                if type(radii) is np.ndarray:
                    r = int(radii[i])
                
                else:            
                    r = radii   
                    
                cv2.circle(imRGB,(col, row), r, color, thickness)
            
        else:
            N = centers.shape[0]
                
            for i in range(0, N, 1):
                col = int(centers[i, 0])
                row = int(centers[i, 1])
                
                if type(radii) is np.ndarray:
                    r = int(radii[i])
                
                else:            
                    r = radii
                
                cv2.circle(imRGB,(col, row), r, color, thickness)
            
    return imRGB

def add_contour(imRGB, mask, color = (255,255,255), thickness = 5):
    contours, hierarchy= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    # segmentPeduncle = self.imRGB.copy()
    cv2.drawContours(imRGB, contours, -1, color, thickness)
    return imRGB

def plot_circles(imRGB, centers, radii = 5, pwd = None, name = None, 
                 figureTitle = "", titleSize = 20, resolution = 300, 
                 fileFormat = 'pdf', color = (255,255,255), thickness = 5):
    
    plt.rcParams["savefig.format"] = fileFormat 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = titleSize
    
    fig = plt.figure() 
    plt.subplot(1, 1, 1)
    
    imRGB = add_circles(imRGB, centers, radii = radii, color = color, thickness = thickness)    
    
    
    plt.imshow(imRGB)
    plt.title(figureTitle)
    plt.axis('off')
    
    if pwd is not None:
        fig.savefig(os.path.join(pwd, name), dpi = resolution, pad_inches=0)
    return fig
    
def pipi(angle):
    # cast angle to range [-180, 180]
    return (angle + 180) % 360 - 180    
    
def change_brightness(img, brightness):
    img_copy = img.copy()
    return img_copy + ((255  - img_copy)**brightness).astype(np.uint8) 