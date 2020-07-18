# -*- coding: utf-8 -*-

## imports ##
import cv2
import numpy as np
import skan

# custom functions
from util import add_circles
from util import save_img
from util import remove_blobs, bin2img, img2bin

from skimage.morphology import skeletonize

def get_node_id(branch_data, skeleton):

    src_node_id = np.unique(branch_data['node-id-src'].values)
    dst_node_id = np.unique(branch_data['node-id-dst'].values)
    all_node_id = np.unique(np.append(src_node_id, dst_node_id))        
    
    deg = skeleton.degrees [all_node_id]
    end_node_index= np.argwhere(deg == 1)[:, 0] # endpoint
    
    end_node_id = all_node_id[end_node_index]        
    junc_node_id = np.setdiff1d(all_node_id,end_node_id)        
    
    return junc_node_id, end_node_id    

def get_node_coord(skeleton_img):
    
    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)        
    
    # get all node IDs
    junc_node_id, end_node_id = get_node_id(branch_data, skeleton)    
    
    # swap cols
    end_node_coord = skeleton.coordinates[end_node_id][:,[1, 0]]
    junc_node_coord = skeleton.coordinates[junc_node_id][:,[1, 0]]

    return junc_node_coord, end_node_coord

def get_locations_on_mask(mask, locations):

    col, row = np.nonzero(mask)
    loc = np.transpose(np.matrix(np.vstack((row, col))))

    iKeep = []
    for i in range(locations.shape[0]):
        location = locations[i,:]
        # col, row = np.nonzero(skeleton)
        dist = np.sqrt(np.sum(np.power(loc - location, 2), 1))
        if np.amin(dist) < 5:
            iKeep.append(i)

    return locations[iKeep, :]
    
def get_attached_branches(branch_data, node_id):
    src = np.argwhere(branch_data['node-id-src'] == node_id)[:, 0]
    dst = np.argwhere(branch_data['node-id-dst'] == node_id)[:, 0]
    branch_indexes = np.unique(np.append(src, dst))
    return branch_indexes
    
    
def get_attached_nodes(branch_data, branch_indexes):
    node_id_src = branch_data['node-id-src'][branch_indexes]
    node_id_dst = branch_data['node-id-dst'][branch_indexes]   
    node_ids = np.unique(np.append(node_id_src, node_id_dst)) 
    return node_ids
    
def get_new_node(branch_data, branch_index, old_node):
    node_id_src = branch_data['node-id-src'][branch_index]
    node_id_dst = branch_data['node-id-dst'][branch_index]   
    if node_id_src == old_node:
        return node_id_dst
    elif node_id_dst == old_node:
        return node_id_src
    else:
        print('Old node was not attached to this branch!')
        return None
    
def update_skeleton(skeleton_img, skeleton, i_remove):
    dtype = skeleton_img.dtype
    max_value = np.iinfo(dtype).max

    skeleton_prune_img = skeleton_img.copy()    
    
    for i in i_remove:
        px_coords = skeleton.path_coordinates(i).astype(int)
        
        for px_coord in px_coords:
            skeleton_prune_img[px_coord[0], px_coord[1]] = 0
    
    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = skeleton_prune_img
    img = cv2.dilate(skeleton_prune_img, kernel, iterations = 1)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = remove_blobs(img)
    return bin2img(skeletonize(img/max_value))

def generate_skeleton_img(skeleton, i_keep, shape, dtype=np.uint8):
    max_value = np.iinfo(dtype).max
    skeleton_img = np.zeros(shape, dtype = dtype)
  
    for i in i_keep:
        px_coords = skeleton.path_coordinates(i).astype(int)
        
        for px_coord in px_coords:
            skeleton_img[px_coord[0], px_coord[1]] = max_value
    
    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = skeleton_img
    img = cv2.dilate(skeleton_img, kernel, iterations = 1)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    return bin2img(skeletonize(img2bin(img)))

def threshold_branch_length(skeleton_img, distance_threshold):
    
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)    
    
    tip_junction = branch_data['branch-type'] == 1
    
    b_remove = (skeleton.distances < distance_threshold) & tip_junction 
    i_remove = np.argwhere(b_remove)[:,0]
    
    return update_skeleton(skeleton_img, skeleton, i_remove)

def filter_branch_length(skeleton_img):
    
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)
    
    max_path = list()      
    max_length = 0
  
    junc_node_ids, start_node_ids = get_node_id(branch_data, skeleton)   
    
    for node_id in start_node_ids:
        
        current_path = list()
        current_length = 0
        current_path, current_length = find_largest_branch(branch_data, skeleton, 
                                               node_id, node_id, current_path, current_length) 
                                              
        if current_length > max_length:
            max_path = current_path
            max_length = current_length
                                              
    return generate_skeleton_img(skeleton, max_path, skeleton_img.shape), max_path
   
   
def find_largest_branch(branch_data, skeleton, node_id, start_node, path, length, 
                        angle = None, max_path = [], max_length = 0, branch_visited = []):
          
    branch_indexes = get_attached_branches(branch_data, node_id)
    branch_indexes = list(set(branch_indexes) - set(branch_visited))
    
    for branch_index in branch_indexes:    
        path_new = path[:]
        branch_visited_new = branch_visited[:]
        length_new = length
        
        branch_visited_new.append(branch_index)
        node_new = get_new_node(branch_data, branch_index, node_id)
        angle_new = node_angle(node_id, node_new, skeleton) 
        
        if angle is None:
            diff = 0
        else:
            diff = abs(angle - angle_new)
        
        if (diff < 45): # or (length < 10):
            # walk over branch
            angle_total = node_angle(start_node, node_new, skeleton)
            path_new.append(branch_index)
            length_new += skeleton.distances[branch_index]
                 
        else:
            # reset path
            start_node = node_id
            path_new = [branch_index]
            angle_total = angle_new 
            length_new = skeleton.distances[branch_index]                
            
        max_path, max_length = find_largest_branch(branch_data, skeleton, 
                                                   node_new,
                                                   start_node,
                                                   path_new,
                                                   length_new,  
                                                   angle = angle_total,
                                                   max_path = max_path,
                                                   max_length = max_length,
                                                   branch_visited = branch_visited_new)
                                                   
    # store as largest branch if required
    if length > max_length:
        max_path = path
        max_length = length                           
                                                        
    return max_path, max_length

def node_angle(node_1_id, node_2_id, skeleton):
    src = skeleton.coordinates[node_1_id][[1,0]]
    dst = skeleton.coordinates[node_2_id][[1,0]]
    angle = np.rad2deg(np.arctan2((dst[0] - src[0]), (dst[1] - src[1])))
    return angle

def visualize_skeleton(bg_img, skeleton_img, junc_coord = None, 
                       end_coord = None, name = "", pwd = None):
                           
    junc_color = (100, 0, 200)
    end_color =   (200, 0, 0)  
    pend_color = (0,150,30)

    if (junc_coord is None) and (end_coord is None):   
        junc_coord, end_coord = get_node_coord(skeleton_img)
    
    elif junc_coord is None:
        junc_coord, _ = get_node_coord(skeleton_img)
        
    elif end_coord is None:
        _, end_coord = get_node_coord(skeleton_img)
    
    img = bg_img.copy()
    contours, hierarchy= cv2.findContours(skeleton_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    cv2.drawContours(img, contours, -1, pend_color, 2)        
    img = add_circles(img, junc_coord, color=junc_color, thickness=-1, radii = 3)
    img = add_circles(img, end_coord, color=end_color, thickness=-1, radii = 3)
    save_img(img, pwd, name) 


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

def detect_peduncle(peduncle_img, distance_threshold, bg_img = None, 
                    save = False, name = "", pwd = ""):
    
    if bg_img is None:
        bg_img = peduncle_img
    
    
    # skeletonize peduncle segment
    skeleton_img = bin2img(skeletonize(img2bin(peduncle_img)))  
    if save:
        visualize_skeleton(bg_img, skeleton_img, name=name+"_01", pwd=pwd)        
    
    # prune all smal branches
    skeleton_img = threshold_branch_length(skeleton_img, distance_threshold) 
    if save:
        visualize_skeleton(bg_img, skeleton_img, name=name+"_02", pwd=pwd)        
        
    # summerize skeleton
    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)        
    
    # get all node coordiantes
    all_juncions, _ = get_node_coord(skeleton_img)
    
    # determine main peduncle
    skeleton_img, i_keep = filter_branch_length(skeleton_img)    
    branch_data = branch_data.loc[i_keep]
    
    # get end points
    _, endpoints = get_node_coord(skeleton_img)
    
    # only select the junctions which lie on the main peduncle
    junctions = get_locations_on_mask(skeleton_img, all_juncions)     
    
    # get the centers of the obtained branches
    branch_center,_ = get_center_branch(branch_data, skeleton_img)
    
    if save:
        visualize_skeleton(bg_img, skeleton_img, junc_coord=junctions, 
                           end_coord = endpoints, name=name+"_03", 
                           pwd=pwd)                      
    if save:
        visualize_skeleton(bg_img, skeleton_img, junc_coord=branch_center, 
                       end_coord = endpoints, name=name+"_04", 
                       pwd=pwd)  
    
    
    
    return skeleton_img, branch_center