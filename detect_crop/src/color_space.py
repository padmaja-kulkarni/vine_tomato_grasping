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



# custom functions
from detect_crop.util import rgb2hsi
from detect_crop.util import romove_blobs_2
from detect_crop.util import segmentation_parametric
from detect_crop.util import segmentation_cluster

from image_processing import visualize_cluster

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
pathCurrent = os.path.dirname(__file__)
dataSet = "tomato_rot"
fileName = "tomato_RGB_001.jpeg"

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", "color_space")

imPath = os.path.join(pwdData, fileName)
imBGR = cv2.imread(imPath)
[H, W] = imBGR.shape[:2]
h = int(H/2)
w = int(W/2)

row = H - h
col = int(w/1.5)
# imBGR = imBGR[row:row + h, col:col + w]

plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

#%% RGB
imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)

colorType = ['Red' ,'Green', 'Blue']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imRGB[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'RGB_' + colorType[i]))


#%% HSL
imHLS = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HLS)
imHSL = np.dstack((np.dstack((imHLS[:,:,0], imHLS[:,:,2])), imHLS[:,:,1]))
colorType = ['Hue' ,'Saturation', 'Lightness']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imHSL[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'HSL_' + colorType[i]))

#%% HSV
imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)

colorType = ['Hue' ,'Saturation', 'Value']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imHSV[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'HSV_' + colorType[i]))
    
#%% HSI
imHSI = rgb2hsi(imRGB)
colorType = ['Hue' ,'Saturation', 'Intensity']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imHSI[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'HSI_' + colorType[i]))    
    
    
#%% LAB
imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)

colorType = ['Lightness' ,'A', 'B']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imLAB[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'LAB_' + colorType[i]))
    
#%% YCrCb
imYCrCb = cv2.cvtColor(imRGB, cv2.COLOR_RGB2YCrCb)

colorType = ['Luminance' ,'Cr', 'Cb']
for i in range(0,3):
    plt.figure()
    plt.axis('off')
    plt.imshow(imYCrCb[:,:,i], vmin=0, vmax=255) 
    plt.title(colorType[i])
    plt.savefig(os.path.join(pwdResults, 'YCrCb_' + colorType[i]))
    
#%% Legend
plt.figure()
plt.yticks([])
plt.xticks([0, 255])
gradient = np.linspace(0, 1, 256)
gradient = np.vstack((gradient, gradient, gradient, gradient, gradient, gradient))
plt.imshow(gradient)
plt.savefig(os.path.join(pwdResults, 'legend'))


#%%

# define criteria and apply kmeans()
background, tomato, peduncle = segmentation_cluster(imRGB, 255)



#%% VISUALIZE
scale = 0.1

height = int(h * scale)
width = int(w * scale)
dim = (width, height)

RGB_mini = cv2.resize(imRGB, dim, interpolation = cv2.INTER_AREA)
HSV_mini = cv2.resize(imHSV, dim, interpolation = cv2.INTER_AREA)

# rgb_3d_scatter(RGB_mini)

# hsv_3d_scatter(HSV_mini, RGB_mini)

hsv_2d_scatter(HSV_mini, RGB_mini)


#%%

# Now separate the data, Note the flatten()
#lab1 = (label.ravel()==0).reshape((h, w))
#lab2 = (label.ravel()==1).reshape((h, w))
# lab3 = (label.ravel()==2).reshape((h, w))


#red = 255 *  lab1 # imRGB[:,:,0] * 
#green = 255 *lab2 # imRGB[:,:,1] * 
#blue = 255 * lab3 # imRGB[:,:,2] * 

#imSEGMENT = np.dstack((np.dstack((red, green)), blue))

plt.figure()
plt.axis('off')
plt.imshow(imRGB) 

visualize_cluster(imRGB, background, tomato, peduncle)