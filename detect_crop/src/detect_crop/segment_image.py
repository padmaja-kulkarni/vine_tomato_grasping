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
from util import save_fig
    
def k_means_hue(img_hue, n_clusters):

    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2*np.float32(img_hue.flatten()))
    data = np.stack((np.cos(angle), np.sin(angle)), axis = 1)
 
    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 1, np.sin(np.deg2rad(5)))
    compactness,labels,centers_xy = cv2.kmeans(data, 
                                               n_clusters, 
                                               None, 
                                               criteria, 
                                               2, 
                                               cv2.KMEANS_PP_CENTERS) 

    # convert the centers from xy to angles
    centers = np.rad2deg(np.arctan2(centers_xy[:, 1], centers_xy[:, 0]))    
    return centers, labels

def segment_truss(img_hue, save = "False", name = "", pwd = ""):
    
    n = 3
    centers, labels = k_means_hue(img_hue, n)
    
    # determine which center corresponds to which segment
    lbl = {}
    lbl["peduncle"] = np.argmax(centers)
    lbl["background"] = np.argmin(centers)
    lbl["tomato"] = list(set(range(0, n)) - set(lbl.values()))[0]
    
    # compute masks
    dim = img_hue.shape
    tomato = label2img(labels, lbl["tomato"], dim)     
    peduncle = label2img(labels, lbl["peduncle"], dim)   
    background = label2img(labels, lbl["background"], dim)  
    
    if save:
        hue_hist(img_hue, centers, lbl, name, pwd)
    
    return background, tomato, peduncle
        
def segment_tomato(img_hue, save = False, name = "", pwd = ""):

    n = 2
    centers, labels = k_means_hue(img_hue, n)
 
    # determine which center corresponds to which segment
    lbl = {}
    lbl["background"] = np.argmin(centers)
    lbl["peduncle"] = None
    lbl["tomato"] = list(set(range(0,n)) - set(lbl.values()))[0]
    
    # compute masks
    dim = img_hue.shape
    tomato = label2img(labels, lbl["tomato"], dim)     
    peduncle = np.zeros(img_hue.shape, dtype = np.uint8)
    background = label2img(labels, lbl["background"], dim)  
    
    if save:
        hue_hist(img_hue, centers, lbl, name, pwd)    
    
    return background, tomato, peduncle, lbl, centers        
        
def hue_hist(img_hue, centers, lbl, name, pwd):
    
    # [-180, 180] => [0, 360]
    centers[centers<0] += 360       
    
    # plot Hue (HSV)
    fig, ax= plt.subplots(1)
    plt.yscale("log")

    center_background = centers[lbl["background"]]
    center_tomato = centers[lbl["tomato"]]
    center_peduncle = centers[lbl["peduncle"]]

    ax.axvline(x= center_background,  color='b')
    ax.axvline(x= center_tomato,  color='r')
    ax.axvline(x= center_peduncle,  color='g')
    
    x0 = 0
    x1 = (center_tomato + center_peduncle)/2
    x2 = (center_peduncle + center_background)/2
    x3 = (center_background + center_tomato + 360)/2
    x4 = 360
    alpha = 0.2
    
    plt.axvspan(x0, x1, color='r', alpha=alpha, lw=0)
    plt.axvspan(x1, x2, color='g', alpha=alpha, lw=0)  
    plt.axvspan(x2, x3, color='b', alpha=alpha, lw=0)
    plt.axvspan(x3, x4, color='r', alpha=alpha, lw=0)  
    
    bins = 180
    angle = img_hue.flatten().astype('uint16')*2
    radii, bins, patches = ax.hist(angle, bins=bins, range=(0, 360), color = "black", lw=0)
    save_fig(fig, pwd, name + "_hist", figureTitle = "", resolution = 100, titleSize = 10)        
        
def label2img(labels, label, dim):
    data = labels.ravel() == label
    img = data.reshape(dim)
    return bin2img(img)           

def hue_scatter(xy, RGB):
    
    rows, cols = RGB.shape[:2]
    
    pixel_colors = RGB.reshape((rows*cols, 3))
    norm = colors.Normalize(vmin=-1.,vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()
    
    fig =  plt.figure()
    ax = fig.add_subplot(2, 2, 1)
    ax.scatter(xy[:, 0], xy[:, 1], facecolors=pixel_colors, marker=".")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    
    plt.show()