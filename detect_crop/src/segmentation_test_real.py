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
from detect_crop.util import save_fig, bin2img

from matplotlib import colors

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

#%% init

# tomato rot: 15
# tomato cases: 48
dataSet = "real_blue"


N = 23              # tomato file to load
extension = ".png"
dataSet = "real_blue" # "tomato_rot" #  

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

for iTomato in range(1, N):

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
        
        img_hue = imHSV[:, :, 0] # hue
        dim = img_hue.shape
        H = dim[0]
        W = dim[1]
 
        background, tomato, peduncle, label_background, label_tomato, label_peduncle, centers = segment_truss_extensive(img_hue, imMax) 
        truss = cv2.bitwise_or(tomato,peduncle)
        
        # [-180, 180] => [0, 360]
        centers[centers<0] += 360   
        
        #%% VISUALIZE
        scale = 0.1
        
        height = int(H * scale)
        width = int(W * scale)
        dim = (width, height)
        
        RGB_mini = cv2.resize(imRGB, dim, interpolation = cv2.INTER_AREA)
        im1_mini = cv2.resize(img_hue, dim, interpolation = cv2.INTER_AREA)    
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
        radii, bins, patches = ax.hist(img_hue.flatten().astype('uint16')*2, bins=N, range=(0, 360), color = "black", lw=0)
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