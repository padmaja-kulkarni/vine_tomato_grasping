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

from skan import skeleton_to_csgraph
from skan import Skeleton, summarize

from matplotlib import pyplot as plt

# custom functions
from detect_crop.util import save_img, add_circles

from detect_crop.util import make_dirs

from detect_crop.ProcessImage import ProcessImage
from skimage.morphology import skeletonize


def get_locations_on_mask(mask, locations):
    
    
    col, row = np.nonzero(mask)
    loc = np.transpose(np.matrix(np.vstack((row, col))))

    iKeep = []
    for i in range(locations.shape[0]):
        location = locations[i,:]
        # col, row = np.nonzero(skeleton)
        dist = np.sqrt(np.sum(np.power(loc - location, 2), 1))
        if np.amin(dist) < 20:
            iKeep.append(i)

    return locations[iKeep, :]

def get_neighbours(row,col, skeleton):
    
    patch_width = 1

    dim = skeleton.shape
    H = dim[0]
    W = dim[1]
    
    row_start = max([row - patch_width, 0])
    row_end = min([row + patch_width, H - 1])
    
    col_start = max([col - patch_width, 0])
    col_end = min([col + patch_width, W - 1])
    
    rows = np.arange(row_start, row_end + 1)
    cols = np.arange(col_start, col_end + 1)
    
    neighbours = skeleton[rows[:, np.newaxis], cols]
    return neighbours


def count_neighbours(row, col, skeleton):
    neighbours = get_neighbours(row,col, skeleton)
    return np.sum(neighbours) - 1    

#def add_circles(imRGB, centers, r = 5, color = (255,255,255), thickness = 5):
#    for center in centers:
#        cv2.circle(imRGB,(center[1], center[0]), r, color, thickness)
#            
#    return imRGB

#%% init
iTomato = 1 #  48 #        # tomato file to load
nDigits = 3

plt.rcParams["image.cmap"] = 'plasma'
plt.rcParams["savefig.format"] = 'pdf' 
plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

pathCurrent = os.path.dirname(__file__)
dataSet = "real_blue" # "tomato_cases" # 

pwdData = os.path.join(pathCurrent, "data", dataSet)
pwdResults = os.path.join(pathCurrent, "results", dataSet, "color_space")

make_dirs(pwdData)
make_dirs(pwdResults)

imMax = 255
count = 0

tomatoID = str(iTomato).zfill(nDigits)
tomatoName = tomatoID # "tomato" + "_RGB_" + 
fileName = tomatoName + ".png" # ".jpg" # 


imPath = os.path.join(pwdData, fileName)
imBGR = cv2.imread(imPath)

imRGB = cv2.cvtColor(imBGR, cv2.COLOR_BGR2RGB)

image = ProcessImage(imRGB, camera_sim = False,
                                 use_truss = True,
                                 tomatoName = 'ros_tomato',
                                 pwdProcess = pwdResults,
                                 saveIntermediate = False)

if not image.process_image():
    print("[OBJECT DETECTION] Failed to process image")

object_features = image.get_object_features()
peduncle_mask = object_features['peduncle']["mask"]
peduncle_mask_main = object_features['peduncle']["mask_main"]

fig = plt.figure()
plt.imshow(peduncle_mask)

skeleton = skeletonize(peduncle_mask/255)

fig = plt.figure()
plt.imshow(skeleton)


pixel_graph0, coordinates0, degrees0 = skeleton_to_csgraph(skeleton)


branch_data = summarize(Skeleton(skeleton))
branch_data.head()

all_nodes_ID = branch_data['node-id-src'].values
deadBranch = branch_data['branch-type'] == 1 # junction-to-endpoint
# junstionSrc = branch_data['node-id-src'][deadBranch].values # from node
end_points_ID = branch_data['node-id-dst'][deadBranch].values # to node

# Prune all nodes which correspond to a branch going from junction to an endpoint
junctions_ID = np.setdiff1d(all_nodes_ID,end_points_ID)
# allJunctions = np.setdiff1d(allJunctions,junctionDst)

junctions = coordinates0[junctions_ID]
end_points = coordinates0[end_points_ID]
# col, row = np.nonzero((degrees0 == 3) & (penduncleMain > 0))
# loc = np.transpose(np.matrix(np.vstack((row, col))))

junctions[:,[0, 1]] = junctions[:,[1, 0]]
end_points[:,[0, 1]] = end_points[:,[1, 0]]

junctions = get_locations_on_mask(peduncle_mask_main, junctions)
end_points = get_locations_on_mask(peduncle_mask_main, end_points)

 
radii_junctions = np.repeat(5, junctions.shape[0])
radii_end_points = np.repeat(5, end_points.shape[0])


img = add_circles(peduncle_mask, junctions, radii_junctions, color = (255/3), thickness = 2)
img = add_circles(img, end_points, radii_end_points, color = (2*255/3), thickness = 2)


fig = plt.figure()
plt.imshow(img)

#index = np.nonzero(skeleton)
#
#end_point = []
#junction = []
#
#for row, col in zip(index[0], index[1]):
#    neighbours = count_neighbours(row,col, skeleton)
#    
#    if neighbours == 1:
#        end_point.append((row, col))
#        
#    if neighbours >= 3:
#        junction.append((row, col))
#        
#
#
#pixel = end_point[0]
#
#
#
#img = add_circles(peduncle_mask, end_point, r = 7, color = (255/3), thickness = 2)
#
#img = add_circles(peduncle_mask, junction, r = 7, color = (255*2/3), thickness = 2)
#
#save_img(img, pwdResults, tomatoName, resolution = 600)