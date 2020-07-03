# -*- coding: utf-8 -*-
"""
Created on Fri May 22 12:04:59 2020

@author: taeke
"""

# -*- coding: utf-8 -*-
"""
Created on Wed May 20 13:32:31 2020

@author: taeke
"""

## imports ##
import os # os.sep
import cv2
import numpy as np

from matplotlib import pyplot as plt

# custom functions
from detect_crop.util import save_img
from detect_crop.util import stack_segments
from detect_crop.util import make_dirs
from detect_crop.util import save_fig, label2img

from matplotlib import colors

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

#%% init

# tomato rot: 15
# tomato cases: 48
dataSet = "real_blue"


settings = {
    "real_blue": {
        "extension": ".png",
        "files": 5,
        "name": "real_blue"},
    "artificial": {
        "extension": ".png",
        "files": 19,
        "name": "artificial"},
    "sim_blue": {
        "extension": ".png",
        "files": 1,
        "name": "sim_blue"},
            }


N = settings[dataSet]["files"]              # tomato file to load
extension = settings[dataSet]["extension"]
dataSet = settings[dataSet]["name"] # "tomato_rot" #  

nDigits = 3
# ls | cat -n | while read n f; do mv "$f" `printf "%03d.jpg" $n`; done


plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

pathCurrent = os.path.dirname(__file__)


pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", dataSet, "segmentation")

make_dirs(pwdData)
make_dirs(pwdResults)

imMax = 255
count = 0

for iTomato in range(1, 23):

    tomatoID = str(iTomato).zfill(nDigits)
    tomatoName = tomatoID # "tomato" + "_RGB_" + 
    fileName = tomatoName + extension
    
    imPath = os.path.join(pwdData, fileName)
    imBGR = cv2.imread(imPath)
    
    if imBGR is None:
        print("Failed to load image from path: %s" %(imPath))
    else:
        


        # color spaces
        imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
        imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
        imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
        
        #%%#################
        ### SEGMENTATION ###
        ####################
        im1 = imHSV[:, :, 0] # hue
 
        dim = im1.shape
        H = dim[0]
        W = dim[1] 
        
        angle = np.deg2rad(2*np.float32(im1.flatten()))
        data = np.stack((np.cos(angle), np.sin(angle)), axis = 1)
 
        # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
        criteria = (cv2.TERM_CRITERIA_EPS, 1, np.sin(np.deg2rad(5))) #  + cv2.TERM_CRITERIA_MAX_ITER
        compactness,labels,centers_xy = cv2.kmeans(data, 
                                                   3, 
                                                   None, 
                                                   criteria, 
                                                   2, 
                                                   cv2.KMEANS_PP_CENTERS) 
 
        centers = np.rad2deg(np.arctan2(centers_xy[:, 1], centers_xy[:, 0]))
        
        label_peduncle = np.argmax(centers)
        label_background = np.argmin(centers)
        label_tomato = list(set([0, 1, 2]) - set([label_peduncle, label_background]))[0]
 
         # compute masks
        tomato = label2img(labels, label_tomato, H ,W)     
        peduncle = label2img(labels, label_peduncle, H ,W)   
        background = label2img(labels, label_background, H ,W)   
        truss = cv2.bitwise_or(tomato,peduncle)
        
        # [-180, 180] => [0, 360]
        centers[centers<0] += 360   
        
        #%% VISUALIZE
        scale = 0.1
        
        height = int(H * scale)
        width = int(W * scale)
        dim = (width, height)
        
        RGB_mini = cv2.resize(imRGB, dim, interpolation = cv2.INTER_AREA)
        im1_mini = cv2.resize(im1, dim, interpolation = cv2.INTER_AREA)    
        data = np.stack((np.cos(np.deg2rad(2*np.float32(im1_mini.flatten()))), 
                         np.sin(np.deg2rad(2*np.float32(im1_mini.flatten())))), axis = 1)
    
        # plot Hue (HSV)
        fig, ax= plt.subplots(1)
        # fig.suptitle('Histogram')
        # ax.set_title('H (HSV)')
#        
        plt.yscale("log")

        # ax.set_ylim(0, 4*np.mean(values[0]))
        
        #ax.set_xlim(0, 255)
        center_peduncle = centers[label_peduncle]
        center_tomato = centers[label_tomato]
        center_background = centers[label_background]
        
        ax.axvline(x= center_peduncle,  color='g')
        ax.axvline(x= center_tomato,  color='r')
        ax.axvline(x= center_background,  color='b')
        
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
        
        
        N = 180
        radii, bins, patches = ax.hist(im1.flatten().astype('uint16')*2, bins=N, range=(0, 360), color = "black", lw=0)
        save_fig(fig, pwdResults, tomatoID + "_hist", figureTitle = "", resolution = 100, titleSize = 10)
        
#        ## circular        
#        width = (2*np.pi) / N
#        bottom = 1
#        theta = np.linspace(0.0, 2 * np.pi, N, endpoint=False)  
#        
#        # ax= plt.subplot(111, polar=True)
#        fig, ax= plt.subplots(1)
#        ax = plt.subplot(111, projection='polar')
#        ax.set_rlim((0.1, 1000.0))
#        ax.set_rscale('log')
#        bars = ax.bar(theta, radii, width=width, bottom=bottom)
    
        segmentsRGB = stack_segments(imRGB, background, truss, np.zeros(tomato.shape, dtype = np.uint8))
    
        figureTitle = ""
        save_img(segmentsRGB, pwdResults, tomatoID + "_img_1", figureTitle = figureTitle)
        
    
        segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
        save_img(segmentsRGB, pwdResults, tomatoID + "_img_2", figureTitle = figureTitle)
    
    count = count + 1
    print("completed image %d out of %d" %(count, N))