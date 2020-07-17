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

from detect_crop.segment_image import segment_truss

if __name__ == '__main__':
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
    
        tomato_ID = str(iTomato).zfill(nDigits)
        tomato_name = tomato_ID
        file_name = tomato_name + extension
        
        img_path = os.path.join(pwdData, file_name)
        imBGR = cv2.imread(img_path)
        
        if imBGR is None:
            print("Failed to load image from path: %s" %(img_path))
            break
    
        # color spaces
        imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
        imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
        
        img_hue = imHSV[:, :, 0] # hue
        dim = img_hue.shape
        H = dim[0]
        W = dim[1]
     
        background, tomato, peduncle = segment_truss(img_hue, imMax, save = True, 
                                                     name = tomato_name, pwd = pwdResults) 
        truss = cv2.bitwise_or(tomato,peduncle)
    
        segmentsRGB = stack_segments(imRGB, background, truss, np.zeros(tomato.shape, dtype = np.uint8))
    
        figureTitle = ""
        save_img(segmentsRGB, pwdResults, tomato_ID + "_img_1", figureTitle = figureTitle)
        
    
        segmentsRGB = stack_segments(imRGB, background, tomato, peduncle)
        save_img(segmentsRGB, pwdResults, tomato_ID + "_img_2", figureTitle = figureTitle)
        
        count = count + 1
        print("completed image %d out of %d" %(count, N))