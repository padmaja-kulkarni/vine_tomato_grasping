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
from detect_crop.util import save_img, add_circles

from detect_crop.util import make_dirs

from detect_crop.ProcessImage import ProcessImage
from skimage.morphology import skeletonize, binary_closing


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
    
    
def prune_branches_off_mask(mask, branch_data):
    
    col, row = np.nonzero(mask)
    loc = np.transpose(np.matrix(np.vstack((row, col))))

    iKeep = []
    for i, row in branch_data.iterrows():
        
        dst_node_coord = [row['coord-dst-{1}'], row['coord-dst-{0}']]
        src_node_coord = [row['coord-src-{1}'], row['coord-src-{0}']]
        

        # col, row = np.nonzero(skeleton)
        dst_dist = np.sqrt(np.sum(np.power(loc - dst_node_coord, 2), 1))
        src_dist = np.sqrt(np.sum(np.power(loc - src_node_coord, 2), 1))
        
        if (np.amin(dst_dist) < 10) & (np.amin(src_dist) < 10):
            iKeep.append(i)

    return iKeep
    
def get_node_coord(branch_data, skeleton):
    # get all node IDs
    src_node_id = np.unique(branch_data['node-id-src'].values)
    dst_node_id = np.unique(branch_data['node-id-dst'].values)
    all_node_id = np.unique(np.append(src_node_id, dst_node_id))
    
    # get dead node IDs
    dead_branch_id = np.argwhere(branch_data['branch-type'] == 1)[:,0]
    dead_node_id = np.unique(branch_data['node-id-dst'].values[dead_branch_id])
    junc_node_id = np.setdiff1d(all_node_id,dead_node_id) 
    
    # swap cols
    dead_node_coord = skeleton.coordinates[dead_node_id][:,[1, 0]]
    junc_node_coord = skeleton.coordinates[junc_node_id][:,[1, 0]]

    return junc_node_coord, dead_node_coord

def get_center_branch(branch_data, skeleton_img):
    
    col, row = np.nonzero(skeleton_img)
    loc = np.transpose(np.matrix(np.vstack((row, col))))
    
    dead_branch_center = []
    junc_branch_center = []    
    
    for i, row in branch_data.iterrows():
        
        dst_node_coord = np.array((row['coord-dst-{1}'], row['coord-dst-{0}']))
        src_node_coord = np.array((row['coord-src-{1}'], row['coord-src-{0}']))

        center_node_coord = (dst_node_coord + src_node_coord)/2

        dist = np.sqrt(np.sum(np.power(loc - center_node_coord, 2), 1))

        i = np.argmin(dist)
        center = [loc[i,0], loc[i,1]]
        
        if row['branch-type'] == 1:
            dead_branch_center.append(center)
            
        else:
            junc_branch_center.append(center)

    return np.array(junc_branch_center), np.array(dead_branch_center)

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

image.process_image()

# PARAMETERS
distance_threshold = 10

# Get peduncle info
object_features = image.get_object_features()
peduncle_mask = image.peduncleL
peduncle_mask_main = object_features['peduncle']["mask_main"]

# create skeleton image
skeleton_img = skeletonize(peduncle_mask/255)

# intiailize for skan
skeleton = skan.Skeleton(skeleton_img)
branch_data = skan.summarize(skeleton)

# get all node coordiantes
junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)

all_node_img = add_circles(peduncle_mask.copy(), junc_node_coord, color = (255/3), thickness = 2)
all_node_img = add_circles(all_node_img, dead_node_coord, color = (2*255/3), thickness = 2)

# find dead branches
dead_branch_index = branch_data['branch-type'] == 1 # junction-to-endpoint


b_remove = (skeleton.distances < distance_threshold) & (branch_data['branch-type'] == 1) 
i_remove = np.argwhere(b_remove)[:,0]

# prune dead branches
skeleton_prune_img = skeleton_img.copy()

# print(skeleton.path_coordinates(29))

for i in i_remove:
    
    px_coords = skeleton.path_coordinates(i).astype(int)
#    
#    junction_coord_0 = branch_data['coord-dst-{0}'].values[i]
#    junction_coord_1 = branch_data['coord-dst-{1}'].values[i]
#    junction_coord = [junction_coord_0, junction_coord_1]
#    b_delete = (px_coords ==junction_coord).all(axis=1)
#    i_delete = np.argwhere(b_delete) # [0, 0]
    
#    px_coords = np.delete(px_coords,(i_delete), axis=0)

    
    for px_coord in px_coords:
        skeleton_prune_img[px_coord[0], px_coord[1]] = False

skeleton_prune_img = skeletonize(skeleton_prune_img)
# binary_closing(skeleton_prune_img, selem = np.ones((3,3)))

# intiailize for skan
skeleton_prune = skan.Skeleton(skeleton_prune_img)
branch_data_prune = skan.summarize(skeleton_prune)

# prune brnaches of main peduncle
iKeep = prune_branches_off_mask(peduncle_mask_main, branch_data_prune)
branch_data_prune = branch_data_prune.loc[iKeep]

# prune small branches
iKeep = branch_data_prune['branch-distance'] > 2
branch_data_prune = branch_data_prune.loc[iKeep]

junc_node_coord, dead_node_coord = get_node_coord(branch_data_prune, skeleton_prune)
junc_branch_center, dead_branch_center = get_center_branch(branch_data_prune, skeleton_img)

# visualize results
all_node_prune_img = add_circles(peduncle_mask.copy(), junc_node_coord, color = (255/3), thickness = 2)
all_node_prune_img = add_circles(all_node_prune_img, dead_node_coord, color = (2*255/3), thickness = 2)
all_node_prune_img = add_circles(all_node_prune_img, junc_branch_center, color = (2*255/3), thickness = -1)
all_node_prune_img = add_circles(all_node_prune_img, dead_branch_center, color = (2*255/3), thickness = -1)


# plot result
fig, axs = plt.subplots(2,3)
axs[0,0].imshow(skeleton_img)
axs[1,0].imshow(skeleton_prune_img)
axs[0,1].imshow(all_node_img)
axs[1,1].imshow(all_node_prune_img)
axs[1,2].imshow(peduncle_mask_main)
