# -*- coding: utf-8 -*-
"""
Created on Wed Jul  1 14:21:38 2020

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
import skan

from matplotlib import pyplot as plt

# custom functions
from detect_crop.util import save_img

from detect_crop.util import make_dirs

from detect_crop.ProcessImage import ProcessImage
from skimage.morphology import skeletonize

from detect_crop.util import add_circles

def get_locations_on_mask(mask, locations):
    
    
    col, row = np.nonzero(mask)
    loc = np.transpose(np.matrix(np.vstack((row, col))))

    iKeep = []
    for i in range(locations.shape[0]):
        location = locations[i,:]
        # col, row = np.nonzero(skeleton)
        dist = np.sqrt(np.sum(np.power(loc - location, 2), 1))
        if np.amin(dist) < 10:
            iKeep.append(i)

    return locations[iKeep, :]
    
         


if __name__ == '__main__':
    
    #%% init

    nDigits = 3
    
    plt.rcParams["image.cmap"] = 'plasma'
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = 20
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "tomato_cases" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_tomato")
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255
    count = 0
    
    for iTomato in range(17,18): #  48 #        # tomato file to load    
        
        tomatoID = str(iTomato).zfill(nDigits)
        tomatoName = tomatoID # "tomato" + "_RGB_" + 
        fileName = tomatoName + ".png" # ".jpg" # 
        
        
        imPath = os.path.join(pwdData, fileName)
        imBGR = cv2.imread(imPath)
        
        imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)
        
        image = ProcessImage(camera_sim = False,
                                         use_truss = True,
                                         tomatoName = 'ros_tomato',
                                         pwdProcess = pwdResults,
                                         saveIntermediate = False)
        
        
        image.add_image(imRGB)    
        
        image.color_space()
        image.segment_truss()
        image.filter_img()
        image.rotate_cut_img()
        image.detect_tomatoes()
        
        # detect tomatoes
        blur_size = (3,3)
        tomato_radius_min = 8
        tomato_radius_max = 4
        tomato_distance_min = tomato_radius_max
        dp = 4
        param1 = 20
        param2 = 50      
        
        
        # segmentImg = image.get_segmented_image(local = True)    
        # tomatoImg = image.get_tomato_visualization(local = True)
        use_segment = True
        imageRGB = image.get_image(local = True)
        
        if use_segment:
            image_tomato, image_peduncle, temp = image.get_segments(local = True)
            image_gray = cv2.bitwise_or(image_tomato, image_peduncle)
        else:
            
            image_gray = imageRGB[:,:,0] # cv2.cvtColor(imageRGB, cv2.COLOR_RGB2GRAY)

        dim = image_gray.shape
        truss_blurred = cv2.GaussianBlur(image_gray, blur_size, 0)
        minR = dim[1]/tomato_radius_min
        maxR = dim[1]/tomato_radius_max
        minDist = dim[1]/tomato_distance_min         
        
        
        circles = cv2.HoughCircles(truss_blurred, cv2.HOUGH_GRADIENT, 
                           dp, minDist, param1 = param1, 
                           param2 = param2, minRadius=minR, 
                           maxRadius=maxR) # [x, y, radius]

        centers =np.matrix(circles[0][:,0:2])
        radii = circles[0][:,2]
        print(tomatoName)
        if use_segment:
            iKeep = []
            N = centers.shape[0]
            for i in range(0, N):
             
                image_empty = np.zeros(imageRGB.shape[0:2], dtype=np.uint8)
                mask = cv2.circle(image_empty,(centers[i,0], centers[i,1]), radii[i], 255, -1)            
                
                res = cv2.bitwise_and(image_gray, mask)
                pixels = np.sum(res == 255)
                total = np.pi*radii[i]**2
                ratio = pixels/total
                print(ratio)
                if ratio > 0.5:
                    iKeep.append(i)
                else:
                    print "pruning tomato"
                    
            centers = centers[iKeep, :]
            radii = radii[iKeep]
#        save_img(segmentImg, pwdResults, tomatoName + '_img')        
        
        image_circles = add_circles(imageRGB, centers, radii, thickness = 2)
        
        if use_segment:
            save_name = "segmented"
        else:
            save_name = "red"
        # save_img(image_circles, pwdResults, tomatoName + "_2") 
#        save_img(tomatoImg, pwdResults, tomatoName + '_tomato') 
        

