# -*- coding: utf-8 -*-
"""
Created on Fri Aug  7 12:16:30 2020

@author: taeke
"""

from matplotlib import pyplot as plt
import cv2
import numpy as np
import os

plt.rcParams["image.cmap"] = 'jet' # gray, hsv
plt.rcParams["savefig.format"] = 'png' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

pwd_current = os.path.dirname(__file__)
pwd_data = os.path.join(pwd_current, 'data', 'artificial')

#%% Legend
width = 1000
height = 20

plt.figure()
plt.yticks([])
plt.xticks([0, width], [0.5, 0.7])
gradient = np.linspace(0, 255, num = width + 1, dtype = np.uint8)
gradient = np.vstack(height * (gradient,))
gradient_color = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)
plt.imshow(gradient_color)
plt.savefig(os.path.join(pwd_data, 'legend'))    
