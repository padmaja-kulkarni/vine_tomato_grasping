# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 14:07:48 2020

@author: taeke
"""

import cv2
import numpy as np

seed = 1
nclusters = 2

np.random.seed(seed) # Get always same random numpys
data = np.random.random(size=(1000, 2)).astype(np.float32)
labels = np.random.randint(nclusters,
                           size=(data.shape[0],1),
                           dtype=np.int32)


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1.0)

compactness, new_labels, center = cv2.kmeans(data=data,
                                   K=nclusters,
                                   bestLabels=labels,
                                   criteria=criteria,
                                   attempts=10,
                                   flags=cv2.KMEANS_USE_INITIAL_LABELS)



