#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May  8 14:08:03 2020

@author: taeke
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 08:52:33 2020

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
from detect_crop.util import segmentation_otsu_test
from detect_crop.util import make_dirs
from detect_crop.util import rgb2hsi




#%% init
N = 11 #  48 #        # tomato file to load
nDigits = 3

plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

pathCurrent = os.path.dirname(__file__)
dataSet = "tomato_real_blue" # "tomato_cases" # 

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", dataSet, "color_space")

make_dirs(pwdData)
make_dirs(pwdResults)

imMax = 255
count = 0

for iTomato in range(1, N + 1):

    tomatoID = str(iTomato).zfill(nDigits)
    tomatoName = tomatoID # "tomato" + "_RGB_" + 
    fileName = tomatoName + ".png" # ".jpg" # 
    
    imPath = os.path.join(pwdData, fileName)
    imBGR = cv2.imread(imPath)
    
    # color spaces
    imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    # imHLS = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HLS)
    # imHSL = np.dstack((np.dstack((imHLS[:,:,0], imHLS[:,:,2])), imHLS[:,:,1]))
    # imHSI = rgb2hsi(imRGB)    
    
    imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)    
    

    save_img(imHSV[:,:,0], pwdResults, tomatoID + "H")
    save_img(imHSV[:,:,1], pwdResults, tomatoID + "S")
    save_img(imLAB[:,:,1], pwdResults, tomatoID + "A")
    # save_img(imLAB[:,:,2], pwdResults, tomatoID + "B")
    # save_img(imHSI[:,:,1], pwdResults, tomatoID + "HSI_1S")
    # save_img(imHSV[:,:,2], pwdResults, tomatoID + "HSV_2V")
    # save_img(imLAB[:,:,0], pwdResults, tomatoID + "_L")
    # save_img(imLAB[:,:,1], pwdResults, tomatoID + "_A")  
