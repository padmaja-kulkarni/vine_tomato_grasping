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
from util import angular_difference
    
def k_means_hue(img_hue, n_clusters, centers = None):

    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2*np.float32(img_hue.flatten()))
    data = np.stack((np.cos(angle), np.sin(angle)), axis = 1)
    
    if centers is not None:
        labels = np.array(assign_labels(img_hue, centers)[:,0], dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS #  + cv2.KMEANS_PP_CENTERS
    else:
        labels = None
        flags = cv2.KMEANS_PP_CENTERS

 
    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 10, np.sin(np.deg2rad(1.0)))
    compactness,labels,centers_xy = cv2.kmeans(data=data, 
                                               K=n_clusters, 
                                               bestLabels=labels, # None, # 
                                               criteria=criteria, 
                                               attempts=1, 
                                               flags=flags) # cv2.KMEANS_PP_CENTERS) # 

    # convert the centers from xy to angles\
    centers_out = {}
    centers_out['angles'] = np.arctan2(centers_xy[:, 1], centers_xy[:, 0]) 
    return centers_out, labels

def k_means_hue_a(img_hue, img_a, n_clusters, centers = None):

    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2*np.float32(img_hue.flatten()))
    a_normalize = normalize_image(img_a).flatten()
    data = np.stack((np.cos(angle), np.sin(angle), a_normalize), axis = 1)
    
    if centers is not None:
        labels = np.array(assign_labels(img_hue, centers)[:,0], dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS #  + cv2.KMEANS_PP_CENTERS
    else:
        labels = None
        flags = cv2.KMEANS_PP_CENTERS

 
    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 10, np.sin(np.deg2rad(1.0)))
    compactness,labels,centers_xy = cv2.kmeans(data=data, 
                                               K=n_clusters, 
                                               bestLabels=labels, # None, # 
                                               criteria=criteria, 
                                               attempts=1, 
                                               flags=flags) # cv2.KMEANS_PP_CENTERS) # 

    # convert the centers from xy to angles\
    centers_out = {}
    centers_out['angles'] = np.arctan2(centers_xy[:, 1], centers_xy[:, 0]) 
    centers_out['a'] = centers_xy[:, 2]
    return centers_out, labels

def normalize_image(img):
    return np.float32(img - np.min(img))/np.float32(np.max(img) - np.min(img))

def assign_labels(img_hue, centers):
    angles = np.deg2rad(2*np.float32(img_hue.flatten()))    
    
    centers = np.matrix(centers)
    angles = np.transpose(np.matrix(angles))
    dist = angular_difference(angles, centers)
    labels = np.array(np.argmin(dist, axis=1))
    return labels

def segment_truss(img_hue, img_a = None, save = "False", name = "", pwd = ""):
    
    n = 3
    center = {}
    center['tomato'] = np.deg2rad(0) # [rad]
    center['peduncle'] = np.deg2rad(90)
    center['background'] = np.deg2rad(240) # [rad]    
    centers = center.values()    
    
    if img_a is None:
        centers, labels = k_means_hue(img_hue, n, centers = centers) # centers
    else:
        centers, labels = k_means_hue_a(img_hue, img_a, n, centers = centers) # centers
    
    # determine which center corresponds to which segment
    lbl = {}
    lbl["tomato"] = np.argmin(angular_difference(centers['angles'], center['tomato']))
    lbl["background"] = np.argmin(angular_difference(centers['angles'], center['background']))
    lbl["peduncle"] = list(set(range(0, n)) - set(lbl.values()))[0]
    
    
    # compute masks
    dim = img_hue.shape 
    tomato = label2img(labels, lbl["tomato"], dim)   
    if np.abs(centers['angles'][lbl["background"]] - centers['angles'][lbl["peduncle"]]) < np.deg2rad(10):
        print "did not detect a peduncle"    
        peduncle = bin2img(np.zeros(dim))
        background = cv2.bitwise_not(tomato)
    else:   
        peduncle = label2img(labels, lbl["peduncle"], dim)   
        background = label2img(labels, lbl["background"], dim)  
    if save:
        hue_hist(img_hue, np.rad2deg(centers['angles']), lbl, name, pwd)
        if img_a is not None:
            a_hist(normalize_image(img_a), centers['a'], lbl, name + '_a', pwd)
    
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
    ax.set_xlabel("hue [$^\circ$]")
    ax.set_ylabel("frequency")    
    save_fig(fig, pwd, name + "_hist", titleSize = 10)        
    
def a_hist(img_a, centers, lbl, name, pwd):
    
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
    x1 = (center_background + center_peduncle)/2
    x2 = (center_peduncle + center_tomato)/2
    x3 = 1
    alpha = 0.2
    
    plt.axvspan(x0, x1, color='b', alpha=alpha, lw=0)
    plt.axvspan(x1, x2, color='g', alpha=alpha, lw=0)  
    plt.axvspan(x2, x3, color='r', alpha=alpha, lw=0) 
    
    bins = 80
    angle = img_a.flatten() # .astype('uint16')
    radii, bins, patches = ax.hist(angle, bins=bins, range=(0, 1), color = "black", lw=0)
    ax.set_xlabel("a")
    ax.set_ylabel("frequency")    
    save_fig(fig, pwd, name + "_hist", titleSize = 10)   
    
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