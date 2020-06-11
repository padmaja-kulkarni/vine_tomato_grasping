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
        if np.amin(dist) < 20:
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
        
        if (np.amin(dst_dist) < 20) & (np.amin(src_dist) < 20):
            iKeep.append(i)

    return iKeep

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


# Get peduncle info
object_features = image.get_object_features()
peduncle_mask = image.peduncleL
peduncle_mask_main = object_features['peduncle']["mask_main"]

# create skeleton image
skeleton_img = skeletonize(peduncle_mask/255)


# intiailize for skan
skeleton = skan.Skeleton(skeleton_img)
branch_data = skan.summarize(skeleton)

# all nodes
node_id_src = branch_data['node-id-src'].values
node_id_dst = branch_data['node-id-dst'].values
node_id_all = np.append(node_id_src, node_id_dst)


all_node_coord = skeleton.coordinates[node_id_all]
all_node_coord = all_node_coord[:,[1, 0]]
radii = np.repeat(5, all_node_coord.shape[0])


all_node_img = add_circles(peduncle_mask.copy(), all_node_coord, radii, color = (255/3), thickness = 2)

# find dead branches
dead_branch_index = branch_data['branch-type'] == 1 # junction-to-endpoint


b_remove = (skeleton.distances < 10) & (branch_data['branch-type'] == 1) 

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

# get all node IDs
src_node_id = np.unique(branch_data_prune['node-id-src'].values)
dst_node_id = np.unique(branch_data_prune['node-id-dst'].values)
all_node_id = np.unique(np.append(src_node_id, dst_node_id))

# get dead node IDs
dead_branch_id = np.argwhere(branch_data_prune['branch-type'] == 1)
dead_node_id = np.unique(branch_data_prune['node-id-dst'].values[dead_branch_id])
junc_node_id = np.setdiff1d(all_node_id,dead_node_id) 

# swap cols
dead_node_coord = skeleton_prune.coordinates[dead_node_id][:,[1, 0]]
junc_node_coord = skeleton_prune.coordinates[junc_node_id][:,[1, 0]]

# visualize results
radii = np.repeat(5, junc_node_coord.shape[0])
all_node_prune_img = add_circles(peduncle_mask.copy(), junc_node_coord, radii, color = (255/3), thickness = 2)

radii = np.repeat(5, dead_node_coord.shape[0])
all_node_prune_img = add_circles(all_node_prune_img, dead_node_coord, radii, color = (2*255/3), thickness = 2)



# plot result
fig, axs = plt.subplots(2,3)
axs[0,0].imshow(skeleton_img)
axs[1,0].imshow(skeleton_prune_img)
axs[0,1].imshow(all_node_img)
axs[1,1].imshow(all_node_prune_img)

axs[1,2].imshow(peduncle_mask_main)

#
## junstionSrc = branch_data['node-id-src'][deadBranch].values # from node
#end_points_ID = branch_data['node-id-dst'][deadBranch].values # to node
#
## Prune all nodes which correspond to a branch going from junction to an endpoint
#junctions_ID = np.setdiff1d(all_nodes_ID,end_points_ID)
## allJunctions = np.setdiff1d(allJunctions,junctionDst)
#
#junctions = coordinates0[junctions_ID]
#end_points = coordinates0[end_points_ID]
## col, row = np.nonzero((degrees0 == 3) & (penduncleMain > 0))
## loc = np.transpose(np.matrix(np.vstack((row, col))))
#
#junctions = junctions[:,[1, 0]]
#end_points = end_points[:,[1, 0]]
#
#junctions = get_locations_on_mask(peduncle_mask_main, junctions)
#end_points = get_locations_on_mask(peduncle_mask_main, end_points)
#
# 
#radii_junctions = np.repeat(5, junctions.shape[0])
#radii_end_points = np.repeat(5, end_points.shape[0])
#
#
#img = add_circles(peduncle_mask, junctions, radii_junctions, color = (255/3), thickness = 2)
#img = add_circles(img, end_points, radii_end_points, color = (2*255/3), thickness = 2)
#
#
#fig = plt.figure()
#plt.imshow(img)

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