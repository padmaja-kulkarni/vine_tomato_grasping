#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 25 16:06:20 2020

@author: jelle
"""

import numpy as np
import os
from matplotlib import pyplot as plt

path = os.path.abspath(os.path.join(os.path.dirname(__file__),".."))
filename = 'Tue Feb 25 15:31:28 2020.npy'
images = np.load(path+'/images/'+filename)

#%%

color_image = images[:,:640,:]
depth_image = images[:,640:,:]

plt.imshow(color_image, interpolation='nearest')
plt.show()
plt.imshow(depth_image, interpolation='nearest')
plt.show()
