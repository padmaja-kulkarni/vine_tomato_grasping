# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 16:42:24 2020

@author: taeke
"""

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
    
#    i_remove = list()
#    for branch_index in branch_data.index:
#        if branch_index not in max_path:
#            i_remove.append(branch_index)
#            
#    skeleton_img = update_skeleton(skeleton_img, skeleton, i_remove)
    
def filter_branch_angle(skeleton_img):
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)
    
    junc_node_ids, end_node_ids = get_node_id(branch_data, skeleton)
    i_remove = list()
        
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
    # save_img(skeleton_img, '/home/taeke/catkin_ws/src/flexcraft_jelle/detect_crop/src/results/real_blue/detect_peduncle', '008' + "_" + str(counter)) 
    return skeleton_img
    
    #def get_attached_branches(branch_data, node_ids):
#    branch_indexes = list()
#    
#    for node_id in node_ids:
#        tmp_src = np.argwhere(branch_data['node-id-src'] == node_id)[:, 0]
#        tmp_dst = np.argwhere(branch_data['node-id-dst'] == node_id)[:, 0]
#        branch_indexes.extend(list(np.unique(np.append(tmp_src, tmp_dst))))
#    
#    branch_indexes = np.array(list(set(branch_indexes)))
#    return branch_indexes
    
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