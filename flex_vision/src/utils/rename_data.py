# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 13:49:44 2020

@author: taeke
"""

import os
from src.detect_truss.util import load_rgb
import cv2


pwd_current = os.path.dirname(__file__) # path to THIS file
data_set = 'real_blue'
pwd_data = os.path.join(pwd_current, 'data', data_set)


# load image
file_name = '001.png'
img_rgb = load_rgb(pwd_data, file_name, horizontal = True)


# check contents folder
contents = os.listdir(pwd_data)

# determine starting number
if not len(contents):
    new_id = 1
else:
    contents.sort()
    file_name = contents[-1]
    file_id = file_name[:3]
    new_id = int(file_id) + 1
    
new_file_name = str(new_id).zfill(3) + '.png'
new_file_path = os.path.join(pwd_data, new_file_name)
    
result = cv2.imwrite(new_file_path, img_rgb)
if result is True:
    print('File saved successfully')
else:
    print('Error in saving file')