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

tomato_color = (255,0,0)
peduncle_color = (0, 255, 0)
junction_color = (255, 0, 255)
end_color = (255, 255, 0)

def make_dirs(pwd):
    if not os.path.isdir(pwd):
        print("New path, creating a new folder: " + pwd)
        os.makedirs(pwd)

def load_rgb(pwd, name, horizontal = True):
    
    #load image
    name_full = os.path.join(pwd, name)
    
    if not os.path.exists(name_full):
        print('Cannot load RGB: path does not exist' + name_full)
        return None
        
    img_bgr = cv2.imread(name_full)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    shape = img_rgb.shape[:2]
    
    # transpose image if required
    if horizontal:
        if shape[0] > shape[1] :
            img_rgb = np.transpose(img_rgb, [1,0,2])
            shape = img_rgb.shape[:2]

    if img_rgb is None:
        print("Failed to load image from path: %s" %(name_full))

    return img_rgb

def bin2img(binary, dtype = np.uint8, copy = False):
    'convets an incomming binary image to the desired data type, defualt: uint8'
    max_value = np.iinfo(dtype).max       
    return binary.astype(dtype, copy = copy) * max_value
    
def img2bin(img, copy = False):
    
    dtype = bool
    return img.astype(dtype, copy = copy) 

def remove_blobs(img_in):
    dtype = img_in.dtype
    value = np.iinfo(dtype).max   
    
    # initialize outgoing image
    img_out = np.zeros(img_in.shape[:2], dtype)
    
    # the extra [-2:] is essential to make it work with varios cv2 ersions
    contours, _ = cv2.findContours(img_in, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    
    # only when the image contains a contour
    if len(contours) != 0:
        # print('Filling largest blob...')
        cnt = max(contours, key=cv2.contourArea)
        cv2.drawContours(img_out, [cnt], -1, value, cv2.FILLED)
        # print('Done!...')
    return cv2.bitwise_and(img_in, img_out)

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
        
def save_fig(fig, pwd, name, resolution = 300, title = "", titleSize = 20, ext = 'png'):
        
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


def add_circles(imRGB, centers, radii = 5, color = (255,255,255), thickness = 5):
    if radii is not None:
        
        if isinstance(centers, list):  
            for i, center in enumerate(centers):
                col = int(center[0])
                row = int(center[1])
                
                if isinstance(radii, (list, np.ndarray)):
                    r = int(radii[i])
                
                else:            
                    r = int(radii)   
                    
                cv2.circle(imRGB,(col, row), r, color, thickness)
            
        else:
            N = centers.shape[0]
                
            for i in range(0, N, 1):
                col = int(centers[i, 0])
                row = int(centers[i, 1])
                
                if isinstance(radii, (list, np.ndarray)):
                    r = int(radii[i])
                
                else:            
                    r = radii
                
                cv2.circle(imRGB,(col, row), r, color, thickness)
            
    return imRGB

def plot_segments(img_rgb, background, tomato, peduncle, pwd=None, 
                  file_name=None, title="", alpha = 0.7):
    
    img_segments = stack_segments(img_rgb, background, tomato, peduncle)
    
    added_image = cv2.addWeighted(img_rgb, 1 - alpha,img_segments,alpha,0)
    added_image = add_contour(added_image, tomato, color = tomato_color, thickness = 1)
    added_image = add_contour(added_image, peduncle, color = peduncle_color, thickness = 1)  
    
    if pwd is not None:
        save_img(added_image, pwd, file_name, figureTitle = title)

def plot_features(img_rgb, tomato, peduncle, grasp,
                  alpha = 0.7, pwd = None, file_name=None, title=""):
    
    img_overlay = np.ones(img_rgb.shape, dtype = np.uint8)
    img_overlay = add_circles(img_overlay, tomato['centers'], radii = tomato['radii'], color = tomato_color, thickness = -1)
    img_overlay = add_circles(img_overlay, peduncle['junctions'], radii = 10, color = junction_color, thickness = -1)    
    img_overlay = add_circles(img_overlay, peduncle['ends'], radii = 10, color = end_color, thickness = -1)  

    added_image = cv2.addWeighted(img_rgb, 1,img_overlay,alpha,0)


    added_image = add_circles(added_image, tomato['centers'], radii = tomato['radii'], color = tomato_color, thickness = 1)
    added_image = add_circles(added_image, peduncle['junctions'], radii = 10, color = junction_color, thickness = 1)
    added_image = add_circles(added_image, peduncle['ends'], radii = 10, color = end_color, thickness = 1)
    
    if pwd is not None:
        save_img(added_image, pwd, file_name, figureTitle = title)

    return added_image

def plot_error(img, centers, error_centers, error_radii = None, pwd = None, name = None, title = "", resolution = 300, title_size = 20, ext = 'png'):

    fig, ax = plt.subplots()
    ax.imshow(img)    
    plt.rcParams["savefig.format"] = ext
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = title_size
        
    
    # fig = plt.figure() 
    plt.imshow(img)
    plt.axis('off')
    plt.title(title)
    
    # https://stackoverflow.com/a/27227718
    plt.gca().set_axis_off()
    plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, 
        hspace = 0, wspace = 0)
    plt.margins(0,0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())
    
   
    
    # [x for _, x in sorted(zip(Y,X), key=lambda pair: pair[0])]
    # centers.sort(key = lambda x: x[1]) 
    if error_radii is not None:
        zipped = zip(centers, error_centers, error_radii)
        zipped.sort(key = lambda x: x[0][1])    
        centers, error_centers, error_radii = zip(*zipped)
    else:
        zipped = zip(centers, error_centers)
        zipped.sort(key = lambda x: x[0][1])    
        centers, error_centers = zip(*zipped)

    
    h, w = img.shape[:2]
    n = len(centers)+ 1
    y_text = 0 # 1.0/n* h
    for i, center in enumerate(centers):
        
        bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="k", lw=0.72)
        kw = dict(arrowprops=dict(arrowstyle="-"),
                  bbox=bbox_props, va="center", size = 10, color='k')             
        
        error_center = error_centers[i]
        if error_center is not None:
            center_error = int(round(error_center))
            
            if error_radii is not None:
                radius_error = int(round(error_centers[i]))
                text = 'center: {c:d} px \nradius: {r:d} px'.format(c=center_error, r= radius_error) # str()
            else:
                text = 'error:  {c:d} px'.format(c=center_error) # str()
        else:
            text = 'false positive'
            kw['bbox']['fc'] = (1, 0.8, 0.8)
            kw['bbox']['ec'] = 'r'
            kw['color']= 'r'
            
        y = center[1]
        x = center[0]
        
            
        align_left = 1.0/5.0*w *0.25
        align_right = 4.0/5.0*w
        x_text = (x <= w/2.0)*align_left + (x > w/2.0)*align_right
        y_text = y_text + 1.0/n* h #  # 1.5*(y - h/2.0)   + h/2.0         
        
        x_diff = x_text - x
        y_diff = y_text - y
        if (x_diff > 0 and y_diff > 0) or (x_diff < 0 and y_diff < 0):
            ang = -45 # np.pi/4
        else:
            ang = 45      
        
        
        connectionstyle = "angle,angleA=0,angleB={}".format(ang)
        kw["arrowprops"].update({"connectionstyle": connectionstyle})
        plt.annotate(text, xy=(x, y), xytext=(x_text, y_text), **kw) #  

    if pwd:
        fig.savefig(os.path.join(pwd, name), dpi = resolution, bbox_inches='tight', pad_inches=0)

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
    
def plot_timer(timer_dict, N = 1, threshold = 0, ignore_key=None, pwd = None, name = 'time', title = 'time'):   
    
    for key in timer_dict.keys():
        
        # remove ignored keys
        if key == ignore_key:
            del timer_dict[key]
            continue
            
        # remove empty keys
        if timer_dict[key] == []:
            del timer_dict[key]
       
    values = np.array(timer_dict.values())
    
    if len(values.shape) == 1:
        values = values/N
    else:
        values = np.mean(values, axis = 1)
        
    labels = np.array(timer_dict.keys())   
    
    values_rel = values/np.sum(values)
    i_keep = (values_rel > threshold)
    i_remove = np.bitwise_not(i_keep)
    
    # if everything is put under others
    if np.all(i_remove == True):
        print("No time to plot!")
        return
    
    labels_keep = labels[i_keep].tolist()
    values_keep = values[i_keep].tolist() 
    
    if np.any(i_remove == True):
        remains = np.mean(values[i_remove])
        values_keep.append(remains)
        labels_keep.append('others')


    l = zip(values_keep, labels_keep)
    l.sort()
    values_keep, labels_keep = zip(*l)
    
    donut(values_keep, labels_keep, pwd = pwd, title = title)

#    fig, ax = plt.subplots()
#    ax.pie(values_keep, labels=labels_keep, autopct=make_autopct(values_keep), startangle=90, labeldistance=1.2)
#    ax.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
#    
#    fig.show()

def donut(data, labels, pwd = None, title = None):
    
    data_rel = data/sum(data)*100    
    
    text = []
    separator = ': '
    for label, value, value_rel in zip(labels, data, data_rel):
        text.append(label + separator + str(int(round(value_rel))) + '% (' + str(int(round(value))) + ' ms)')
    
    fig, ax = plt.subplots(figsize=(6, 3), subplot_kw=dict(aspect="equal"))
    
    
    
    wedges, texts = ax.pie(data, wedgeprops=dict(width=0.5), startangle=-45)
    
    bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="k", lw=0.72)
    kw = dict(arrowprops=dict(arrowstyle="-"),
              bbox=bbox_props, zorder=0, va="center")
    
    for i, p in enumerate(wedges):
        ang = (p.theta2 - p.theta1)/2. + p.theta1
        y = np.sin(np.deg2rad(ang))
        x = np.cos(np.deg2rad(ang))
        horizontalalignment = {-1: "right", 1: "left"}[int(np.sign(x))]
        connectionstyle = "angle,angleA=0,angleB={}".format(ang)
        kw["arrowprops"].update({"connectionstyle": connectionstyle})
        ax.annotate(text[i], xy=(x, y), xytext=(1.35*np.sign(x), 1.4*y),
                    horizontalalignment=horizontalalignment, **kw)
    
    ax.set_title(title)

    if pwd is not None:
        save_fig(fig, pwd, 'time')
    
def make_autopct(values):
    def my_autopct(pct):
        total = sum(values)
        val = int(round(pct*total/100.0)) # [ms]
        return '{p:.2f}%  ({v:d} ms)'.format(p=pct,v=val)
    return my_autopct