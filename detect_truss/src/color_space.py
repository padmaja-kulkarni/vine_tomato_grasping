#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 14:13:21 2020

@author: taeke
"""



## imports ##
import os # os.sep
import cv2
import numpy as np

from matplotlib import pyplot as plt


from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from detect_truss.util import save_img


# custom functions
from detect_truss.color_space import rgb2hsi
# from detect_truss.util import romove_blobs_2
# from detect_truss.util import segmentation_parametric
# from detect_truss.util import segmentation_cluster
# from detect_truss.util import save_img
#
# from detect_truss.util import stack_segments

def rgb_3d_scatter(RGB):
    # https://realpython.com/python-opencv-color-spaces/
    R, G, B = cv2.split(RGB)
    rows, cols = RGB.shape[:2]
    
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    
    pixel_colors = RGB.reshape((rows*cols, 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    
    axis.scatter(R.flatten(), G.flatten(), B.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Red")
    axis.set_ylabel("Green")
    axis.set_zlabel("Blue")
    plt.show()

def hsv_3d_scatter(HSV, RGB):
    
    H, S, V = cv2.split(HSV)
    rows, cols = HSV.shape[:2]
    

    pixel_colors = RGB.reshape((rows*cols, 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1, projection="3d")
    axis.scatter(H.flatten(), S.flatten(), V.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Hue")
    axis.set_ylabel("Saturation")
    axis.set_zlabel("Value")
    plt.show()
    
    fig = plt.figure()
    axis = plt.axes()
    axis.scatter(S.flatten(), V.flatten(), facecolors=pixel_colors, marker=".")
    axis.set_xlabel("Saturation")
    axis.set_ylabel("Value")
    plt.show()
    
def hsv_2d_scatter(HSV, RGB):
    
    H, S, V = cv2.split(HSV)
    rows, cols = HSV.shape[:2]
    

    
    pixel_colors = RGB.reshape((rows*cols, 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    
    fig =  plt.figure()
    ax = fig.add_subplot(2, 2, 1)
    ax.scatter(H.flatten(), S.flatten(), facecolors=pixel_colors, marker=".")
    ax.set_xlabel("Hue")
    ax.set_ylabel("Saturation")

    ax = fig.add_subplot(2, 2, 2)
    ax.scatter(H.flatten(), V.flatten(), facecolors=pixel_colors, marker=".")
    ax.set_xlabel("Hue")
    ax.set_ylabel("Value")

    ax = fig.add_subplot(2, 2, 3)    
    ax.scatter(H.flatten(), S.flatten(), facecolors=pixel_colors, marker=".")
    ax.set_xlabel("Saturation")
    ax.set_ylabel("Value")
    
    ax = fig.add_subplot(2, 2, 4, projection='3d')
    ax.scatter(H.flatten(), S.flatten(), V.flatten(), facecolors=pixel_colors, marker=".")
    ax.set_xlabel("Hue")
    ax.set_ylabel("Saturation")
    ax.set_zlabel("Value")


    
    plt.show()
     
    
#%% init
drive = "backup"  # "UBUNTU 16_0"  #
pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data",
                        "detect_truss")
dataset = "lidl"  # "real_blue"

pwd_data = os.path.join(pwd_root, "data", dataset)

file_name = "001.png"

pwd_results = os.path.join(pwd_root, "results", 'color_space')

pwd_img = os.path.join(pwd_data, file_name)
imBGR = cv2.imread(pwd_img)
[H, W] = imBGR.shape[:2]
h = int(H/2)
w = int(W/2.5)

row = int(H/6)
col = int(W/3)
imBGR = imBGR[row:row + h, col:col + w]

plt.rcParams["image.cmap"] = 'plasma' # gray, hsv
plt.rcParams["savefig.format"] = 'png'
plt.rcParams["savefig.bbox"] = 'tight'
plt.rcParams['axes.titlesize'] = 20
title_size = 25

#%% RGB
imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
plt.figure()
plt.axis('off')
plt.imshow(imRGB)
save_img(imRGB, pwd_results, 'RGB')


colorType = ['Red' ,'Green', 'Blue']
for i in range(0,3):
    save_img(imRGB[:, : , i], pwd_results, 'RGB_'+colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)


#%% HSL
imHLS = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HLS)
imHSL = np.dstack((np.dstack((imHLS[:,:,0], imHLS[:,:,2])), imHLS[:,:,1]))
colorType = ['Hue' ,'Saturation', 'Lightness']
for i in range(0,3):
    save_img(imHSL[:, :, i], pwd_results, 'HSL_' + colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)

#%% HSV
imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)

colorType = ['Hue' ,'Saturation', 'Value']
for i in range(0,3):
    save_img(imHSV[:, :, i], pwd_results, 'HSV_' + colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)
    
#%% HSI
imHSI = rgb2hsi(imRGB)
colorType = ['Hue', 'Saturation', 'Intensity']
for i in range(0,3):
    save_img(imHSI[:, :, i], pwd_results, 'HSI_' + colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)
    
    
#%% LAB
imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)

colorType = ['Lightness', 'a*', 'b*']
for i in range(0, 3):
    save_img(imLAB[:, :, i], pwd_results, 'LAB_' + colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)
    
#%% YCrCb
imYCrCb = cv2.cvtColor(imRGB, cv2.COLOR_RGB2YCrCb)

colorType = ['Luminance' ,'Cr', 'Cb']
for i in range(0,3):
    save_img(imYCrCb[:, :, i], pwd_results, 'YCrCb_' + colorType[i], title=colorType[i], vmin=0, vmax=255, title_size=title_size)
    
#%% Legend
plt.figure()
plt.yticks([])
plt.xticks([0, 255])
gradient = np.linspace(0, 1, 256)
gradient = np.vstack((gradient, gradient, gradient, gradient, gradient, gradient))
plt.imshow(gradient)
plt.savefig(os.path.join(pwd_results, 'legend'))


#%% VISUALIZE
# scale = 0.1
#
# height = int(h * scale)
# width = int(w * scale)
# dim = (width, height)
#
# RGB_mini = cv2.resize(imRGB, dim, interpolation = cv2.INTER_AREA)
# HSV_mini = cv2.resize(imHSV, dim, interpolation = cv2.INTER_AREA)

# rgb_3d_scatter(RGB_mini)

# hsv_3d_scatter(HSV_mini, RGB_mini)

# hsv_2d_scatter(HSV_mini, RGB_mini)


#%%

# Now separate the data, Note the flatten()
#lab1 = (label.ravel()==0).reshape((h, w))
#lab2 = (label.ravel()==1).reshape((h, w))
# lab3 = (label.ravel()==2).reshape((h, w))


#red = 255 *  lab1 # imRGB[:,:,0] * 
#green = 255 *lab2 # imRGB[:,:,1] * 
#blue = 255 * lab3 # imRGB[:,:,2] * 

#imSEGMENT = np.dstack((np.dstack((red, green)), blue))