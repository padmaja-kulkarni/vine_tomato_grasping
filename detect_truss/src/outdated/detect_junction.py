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
from detect_crop.util import add_circles

from detect_crop.util import make_dirs

from detect_crop.ProcessImage import ProcessImage
from skimage.morphology import skeletonize

from detect_crop.util import prune_branches_off_mask
from detect_crop.util import get_node_coord
from detect_crop.util import get_center_branch

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
    iTomato = 5 #  48 #        # tomato file to load
    nDigits = 3
    
    plt.rcParams["image.cmap"] = 'plasma'
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = 20
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "tomato_cases" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_junction")
    
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
    image.detect_peduncle()
    
    # Get peduncle info
    # object_features = image.get_object_features()
    peduncle_mask = image.get_peduncle_local()
    peduncle_mask_main = image.penduncleMain
    
    # PARAMETERS
    distance_threshold = 10
    
    # create skeleton image
    skeleton_img = skeletonize(peduncle_mask/255)
    
    # intiailize for skan
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)
    
    # get all node coordiantes
    junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
    
    all_node_img = add_circles(peduncle_mask.copy(), junc_node_coord, color = (255/3), thickness = 2)
    all_node_img = add_circles(all_node_img, dead_node_coord, color = (2*255/3), thickness = 2)
    
    
    b_remove = (skeleton.distances < distance_threshold) & (branch_data['branch-type'] == 1) 
    i_remove = np.argwhere(b_remove)[:,0]
    
    # prune dead branches
    skeleton_prune_img = skeleton_img.copy()
    
    # update skeleton
    for i in i_remove:
        
        px_coords = skeleton.path_coordinates(i).astype(int)
        
        for px_coord in px_coords:
            skeleton_prune_img[px_coord[0], px_coord[1]] = False
    
    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_img = cv2.dilate(skeleton_prune_img.astype(np.uint8), kernel, iterations = 1)
    
    # skeletonize
    skeleton_img_2 = skeletonize(close_img)
    skeleton_prune = skan.Skeleton(skeleton_img_2)
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
    
    
    # save_fig(fig, pwd, name, resolution = 300, figureTitle = "", titleSize = 20, saveFormat = 'png')    
    fig = plt.figure()    
    plt.imshow(imRGB)
    plt.axis('off')
    
    # plot result
    fig, axs = plt.subplots(2,2)
    axs[0,0].imshow(skeleton_img)
    axs[0,0].axis('off')
    
    axs[1,0].imshow(skeleton_img_2)
    axs[1,0].axis('off')
    
    axs[0,1].imshow(all_node_img)
    axs[0,1].axis('off')
    
    axs[1,1].imshow(all_node_prune_img)
    axs[1,1].axis('off')

#    axs[1,2].imshow(peduncle_mask_main)
#    axs[1,2].axis('off')
#
#    axs[0,2].axis('off')    

    fig.savefig(os.path.join(pwdResults, fileName), dpi = 300, bbox_inches='tight', pad_inches=0)
    
def detect_junction(self):

    # create skeleton image
    peduncleL = self.crop(self._peduncle).get_data()
    skeleton_img = skeletonize(peduncleL/self.imMax)
    
    # intiailize for skan
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)
    
    # get all node coordiantes
    junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
    
    b_remove = (skeleton.distances < self.distance_threshold) & (branch_data['branch-type'] == 1) 
    i_remove = np.argwhere(b_remove)[:,0]
    
    # prune dead branches
    skeleton_prune_img = skeleton_img.copy()
    
    # update skeleton
    for i in i_remove:
        
        px_coords = skeleton.path_coordinates(i).astype(int)
        
        for px_coord in px_coords:
            skeleton_prune_img[px_coord[0], px_coord[1]] = False
    
    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_img = cv2.dilate(skeleton_prune_img.astype(np.uint8), kernel, iterations = 1)
    
    # skeletonize
    skeleton_img_2 = skeletonize(close_img)
    skeleton_prune = skan.Skeleton(skeleton_img_2)
    branch_data_prune = skan.summarize(skeleton_prune)
    
    # prune brnaches of main peduncle
    iKeep = prune_branches_off_mask(self.penduncleMain, branch_data_prune)
    branch_data_prune = branch_data_prune.loc[iKeep]
    
    # prune small branches
    iKeep = branch_data_prune['branch-distance'] > 2
    branch_data_prune = branch_data_prune.loc[iKeep]
    
    junc_node_coord, dead_node_coord = get_node_coord(branch_data_prune, skeleton_prune)
    junc_branch_center, dead_branch_center = get_center_branch(branch_data_prune, skeleton_img)
    
    self.junc_branch_center = junc_branch_center


    if self.saveIntermediate:
        plot_circles(self.crop(self._image_RGB).get_data(), junc_branch_center, 5, savePath = self.pwdProcess, saveName = '05_c')