# -*- coding: utf-8 -*-
"""
Created on Thu Jul  2 12:00:49 2020

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
from detect_crop.util import add_circles

from detect_crop.util import make_dirs

from detect_crop.ProcessImage import ProcessImage
from skimage.morphology import skeletonize

from detect_crop.util import save_img
from detect_crop.util import get_node_coord, get_node_id

from detect_crop.util import romove_blobs, bin2img, pipi


def update_skeleton(skeleton_img, skeleton, i_remove):

    skeleton_prune_img = skeleton_img.copy()    
    
    for i in i_remove:
        px_coords = skeleton.path_coordinates(i).astype(int)
        
        for px_coord in px_coords:
            skeleton_prune_img[px_coord[0], px_coord[1]] = 0
    
    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = cv2.dilate(skeleton_prune_img, kernel, iterations = 1)
    img = romove_blobs(img, 255)
    return bin2img(skeletonize(img/255))

def threshold_branch_length(skeleton_img, skeleton, distance_threshold):
    
    branch_data = skan.summarize(skeleton)    
    
    tip_junction = branch_data['branch-type'] == 1
    
    b_remove = (skeleton.distances < distance_threshold) & tip_junction 
    i_remove = np.argwhere(b_remove)[:,0]
    
    return update_skeleton(skeleton_img, skeleton, i_remove)

def threshold_branch_angle(skeleton_img, skeleton, angle_mean, angle_amplitude):

    lb = angle_mean - angle_amplitude    
    ub = angle_mean + angle_amplitude

    branch_data = skan.summarize(skeleton)
    # remove ner vertical branches
        
    ndim = 2
    i_remove = list()
    for i in range(0, skeleton.n_paths):
        src = list()
        dst = list()
        for ii in range(ndim):
            src.append(branch_data['image-coord-src-{%s}' %ii][i])
            dst.append(branch_data['image-coord-dst-{%s}' %ii][i])
            
        angle = np.arctan((dst[0] - src[0])/(dst[1] - src[1]))

        if angle < lb or angle > ub:
            i_remove.append(i)
            
    return update_skeleton(skeleton_img, skeleton, i_remove)

def filter_branch_angle(skeleton_img):
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)
    
    junc_node_ids, end_node_ids = get_node_id(branch_data, skeleton)
    i_remove = list()
    
        # for junc_node_id in junc_node_ids:
    for junc_node_id in junc_node_ids:
        branch_index_src = np.argwhere(branch_data['node-id-src'] == junc_node_id)[:, 0]
        branch_index_dst = np.argwhere(branch_data['node-id-dst'] == junc_node_id)[:, 0]
        branch_indexes = np.unique(np.append(branch_index_src, branch_index_dst))            
    
        angles = list()
        distances = list()
        for branch_index in branch_indexes: # branch_data.iterrows():
            coord_1 = list()
            coord_2 = list()
            for ii in range(2):                 
                coord_1.append(branch_data['image-coord-src-{%s}' %ii][branch_index])
                coord_2.append(branch_data['image-coord-dst-{%s}' %ii][branch_index])                    
                
            if branch_data['node-id-src'][branch_index] == junc_node_id:
                src = coord_1
                dst = coord_2
            else:
                src = coord_2
                dst = coord_1      
                
            angle = np.arctan2((dst[0] - src[0]), (dst[1] - src[1]))/np.pi*180
            distance = skeleton.distances[branch_index]
            
            angles.append(angle)
            distances.append(distance)
            
        if False:
            # indexes to keep
            main_index = np.argmax(distances)
            
            orientation = angles[main_index]
            target = pipi(orientation + 180)

            
            angle_dist = pipi(target - angles)
            next_index = np.argmin(np.abs(angle_dist))
            
            branch_index_keep = (branch_indexes[main_index], branch_indexes[next_index])
        else:
            angles = np.array(angles)
            n = angles.size
            dist_mat = np.empty((n,n))
            
            for i, angle in enumerate(angles):
                target = pipi(angle + 180)
                
                angle_dist = pipi(target - angles)
                dist_mat[i, :] = np.abs(angle_dist)
            
            branch_keep = np.unravel_index(dist_mat.argmin(), dist_mat.shape)
            branch_index_keep = (branch_indexes[branch_keep[0]], branch_indexes[branch_keep[1]])

        # remove others
        for branch_index in branch_indexes:
            if branch_index not in branch_index_keep:
                i_remove.append(branch_index)
                
        print(i_remove)
    skeleton_img = update_skeleton(skeleton_img, skeleton, i_remove)
        
    return skeleton_img

if __name__ == '__main__':
    
    #%% init
     #  48 #        # tomato file to load
    nDigits = 3
    
    plt.rcParams["image.cmap"] = 'plasma'
    plt.rcParams["savefig.format"] = 'pdf' 
    plt.rcParams["savefig.bbox"] = 'tight' 
    plt.rcParams['axes.titlesize'] = 20
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "real_blue" # 
    
    pwdData = os.path.join(pathCurrent, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, "results", dataSet, "detect_peduncle")
    
    make_dirs(pwdData)
    make_dirs(pwdResults)
    
    imMax = 255
    count = 0
    N = 3
    for iTomato in range(1,23):
        
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
    
        distance_threshold = 10
        tomatoes = image.get_tomatoes(local = True)
        
        segment_img = image.get_segmented_image(local = True)   
        peduncle_img = image.get_peduncle_image()
        
        # create skeleton image
        skeleton_img = bin2img(skeletonize(peduncle_img/255))  
        skeleton_img = romove_blobs(skeleton_img, 255)
        
        # intiailize for skan
        skeleton = skan.Skeleton(skeleton_img/255)
        branch_data = skan.summarize(skeleton)    
        

        # get all node coordiantes
        junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
        
        all_node_img = skeleton_img.copy()
        all_node_img = add_circles(all_node_img, junc_node_coord, color = (255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, dead_node_coord, color = (2*255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, tomatoes['pixel'], color = (2*255/3), thickness = -1)
        save_img(all_node_img, pwdResults, tomatoName + "_01") 
        
        
        skeleton_img = threshold_branch_length(skeleton_img, skeleton, distance_threshold)        
        
        # skeletonize
        skeleton = skan.Skeleton(skeleton_img)
        branch_data = skan.summarize(skeleton)
        
        # get all node coordiantes
        junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
        
        
        all_node_img = skeleton_img.copy()
        all_node_img = add_circles(all_node_img, junc_node_coord, color = (255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, dead_node_coord, color = (2*255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, tomatoes['pixel'], color = (2*255/3), thickness = -1)
        save_img(all_node_img, pwdResults, tomatoName + "_02")
        
#        angle_mean = 0
#        angle_amplitude = np.pi/4
#        skeleton_img = threshold_branch_angle(skeleton_img, skeleton, angle_mean, angle_amplitude)
#        
#        # skeletonize
#        skeleton = skan.Skeleton(skeleton_img)
#        branch_data = skan.summarize(skeleton)
#        
#        # get all node coordiantes
#        junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
#        
#        node_img = skeleton_img.copy()
#        node_img = add_circles(node_img, junc_node_coord, color = (255/3), thickness = 1)
#        node_img = add_circles(node_img, dead_node_coord, color = (2*255/3), thickness = 1)
#        node_img = add_circles(node_img, tomatoes['pixel'], color = (2*255/3), thickness = -1)
#        save_img(node_img, pwdResults, tomatoName + "_03")   


        skeleton_img = filter_branch_angle(skeleton_img)

        # skeletonize
        skeleton = skan.Skeleton(skeleton_img)
        branch_data_prune = skan.summarize(skeleton)
        
        # get all node coordiantes
        junc_node_coord, dead_node_coord = get_node_coord(branch_data_prune, skeleton)
        
        
        all_node_img = skeleton_img.copy()
        all_node_img = add_circles(all_node_img, junc_node_coord, color = (255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, dead_node_coord, color = (2*255/3), thickness = 1)
        all_node_img = add_circles(all_node_img, tomatoes['pixel'], color = (2*255/3), thickness = -1)
        save_img(all_node_img, pwdResults, tomatoName + "_04")
        
        all_node_img = skeleton_img.copy()
        contours, hierarchy= cv2.findContours(all_node_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        cv2.drawContours(segment_img, contours, -1, (0,255,0), 3)
        save_img(segment_img, pwdResults, tomatoName)