#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 17:05:38 2020

@author: taeke
"""

## imports ##
import os # os.sep

# custom functions
from detect_crop.util import save_img
from detect_crop.util import stack_segments
from detect_crop.util import make_dirs
from detect_crop.util import load_rgb
from detect_crop.filter_segments import filter_segments

from detect_crop.ProcessImage import ProcessImage

if __name__ == '__main__':
    pwd_current = os.path.dirname(__file__)
    dataset = "real_blue"
    imMax = 255
    
    nDigits = 3
    
    pwd_data = os.path.join(pwd_current, "data", dataset)
    pwd_results = os.path.join(pwd_current, "results", dataset, "morphological")
    
    # create folder if required
    make_dirs(pwd_results)
    
    for iTomato in range(1, 2):
    
        
        tomato_name = str(iTomato).zfill(nDigits) 
        file_name = tomato_name + ".png" # png
            
        img_rgb, DIM = load_rgb(pwd_data, file_name, horizontal = True)
        
        image = ProcessImage(use_truss = True, 
                             name = tomato_name, 
                             pwd = pwd_results,
                             save = False)
        
        image.add_image(img_rgb)    
        image.color_space()
        image.segment_truss()    
        tomato, peduncle, background = image.get_segments(local = False)
        
        tomato_f, peduncle_f, background_f = filter_segments(tomato, peduncle, background, imMax)
        
        # original
        segmentsRGB = stack_segments(img_rgb, background, tomato, peduncle)
        save_img(segmentsRGB, pwd_results, tomato_name + '_1')
        
        
        # save filtered
        segmentsRGB = stack_segments(img_rgb, background_f, tomato_f, peduncle_f)
        save_img(segmentsRGB, pwd_results, tomato_name + '_2')
        
#        H = DIM[0]
#        W = DIM[1]
#        
#        # Cropping, only works for this specific image!
#        h = int(H/2)
#        w = int(W/2)
#        
#        row = int(H/6)
#        col = int(W/3)
#        imRGB = imRGB[row:row + h, col:col + w]