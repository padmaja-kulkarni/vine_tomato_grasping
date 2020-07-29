# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:22:48 2020

@author: taeke
"""


import os
import json
import numpy as np
from util import load_rgb, plot_features, plot_error, make_dirs, change_brightness

def remove_none_from_list(lst_none):
    return [x for x in lst_none if x is not None]

def euclidean_dist(p1, p2):
    'compute the euclidean distance between point_1 and point_2'
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5

def euclidean_dist_list(p1, lst):
    dist = []
    for p2 in lst:
        dist.append(euclidean_dist(p1, p2))
        
    return dist
    
    
def index_true_positives(centers, centers_key):
    
    i_true = []
    for center in centers:
        dist = euclidean_dist_list(center, centers_key)
        i_true.append(np.argmin(dist))
        
    i_false = list(set(range(len(centers_key))) -  set(i_true))  
    return i_true, i_false
    


i_start = 1
i_end = 21
N = i_end - i_start

pwd_current = os.path.dirname(__file__)
dataset = 'depth_blue' # 'real_blue' # "drawing" #  "empty" # "artificial" # 

pwd_lbl = os.path.join(pwd_current, "..", "data", dataset)
pwd_res = os.path.join(pwd_current, "..", "results", dataset, 'json')
pwd_store = os.path.join(pwd_current, "..", "results", dataset, 'final')

make_dirs(pwd_store)

tomato_error_all = {}
junction_error_all = {}

for count, i_truss in enumerate(range(i_start, i_end)):
    print("Analyzing image %d out of %d" %(count + 1, N))

    truss_name = str(i_truss).zfill(3)
    
    file_lbl = os.path.join(pwd_lbl, truss_name + '.json')
    file_inf = os.path.join(pwd_lbl, truss_name + '_info.json')
    file_res = os.path.join(pwd_res, truss_name + '.json')
    
    # load data
    img_rgb = load_rgb(pwd_lbl, truss_name + '.png', horizontal = False)
    
    if not os.path.exists(file_lbl):
        print('Labels do not exist for image: ' + truss_name + ' skipping this file')
        continue
    
    if not os.path.exists(file_inf):
        print('Info does not exist for image: ' + truss_name + ' continueing without info. THIS MAY YIELD STRANGE RESULTS!')
        use_mm = False
    else:
        use_mm = True
    
    with open(file_lbl, "r") as read_file:
        data_lbl = json.load(read_file) 
        
    data_lbl.pop('imageData', None)
    shapes = data_lbl['shapes']
    
    tomato_lbl = {'radii': [], 'centers': []}
    peduncle_lbl = {'junctions': [], 'ends': []}
    
    for shape in shapes:
        
        label = shape['label']
        shape_type = shape['shape_type']
        if label == 'tomato':
            if  shape_type != 'circle':
                print("I do not know what to do with ", label, " of shape type ", shape_type) 
                
            points = shape['points']
            center = points[0]
            radius = euclidean_dist(points[0], points[1])
            
            tomato_lbl['centers'].append(center)
            tomato_lbl['radii'].append(radius)
            
        elif label == 'junction':
            if  shape_type != 'point':
                print("I do not know what to do with ", label, " of shape type ", shape_type)
                
            point = shape['points'][0]
            peduncle_lbl['junctions'].append(point)
    
        elif label == 'end_point' or label == 'end' :
            if shape_type != 'point':
                print("I do not know what to do with ", label, " of shape type ", shape_type)
                
            point = shape['points'][0]
            peduncle_lbl['ends'].append(point)
            
        else:
            print "i do not know what to do with ", label
        
    if use_mm :
        with open(file_inf, "r") as read_file:
            data_inf = json.load(read_file) 
        unit = '[mm]'
        px_per_mm = data_inf['px_per_mm']

    else:
        unit = '[px]'
        px_per_mm = 1 

    # compute com
    radii = np.array(tomato_lbl['radii'])
    centers = np.matrix(tomato_lbl['centers'])
    com = (radii**3) * centers/(np.sum(radii**3))
    tomato_lbl['com'] = com
    
    #img_rgb = change_brightness(img_rgb.copy(), -0.9) # change_brightness( , 0.8)
    img_lbl = img_rgb.copy()
    plot_features(img_lbl, tomato_lbl, peduncle_lbl,  pwd = pwd_store, file_name=truss_name + '_lbl')
    
    with open(file_res, "r") as read_file:
        data_results = json.load(read_file)   
        
    img_res = img_rgb.copy()   
    tomato_res = data_results['tomato']
    peduncle_res = data_results['peduncle']
    
    # TODO: false negative!?
    i_true, i_false = index_true_positives(tomato_lbl['centers'], tomato_res['centers'])
        
    tomato_centers_exists = np.array(tomato_res['centers'])[i_true].tolist()
    tomato_centers_false  = np.array(tomato_res['centers'])[i_false].tolist()  
    
    tomato_radii_exists  = np.array(tomato_res['radii'])[i_true].tolist()
    tomato_radii_false  = np.array(tomato_res['radii'])[i_false].tolist()  
    
    tomato_res['centers'] = tomato_centers_exists
    tomato_res['centers'].extend(tomato_centers_false)
    
    tomato_res['radii'] = tomato_radii_exists
    tomato_res['radii'].extend(tomato_radii_false)
        
    true_pos = len(i_true)
    false_pos = len(i_false)
    labeled_pos = len(tomato_lbl['centers'])
    predict_pos = len(tomato_res['centers'])
    
    com_error = euclidean_dist(tomato_lbl['com'].tolist()[0], tomato_res['com'][0])/px_per_mm
    
    # compute error
    tomato_error = {'radii': [], 'centers': [], 'com': com_error, 'true_pos': true_pos, 'false_pos': false_pos, 'labeled_pos': labeled_pos, 'predict_pos': predict_pos}
    for center_lbl, center_res in zip(tomato_lbl['centers'], tomato_res['centers']):
        dist = euclidean_dist(center_lbl, center_res)
        tomato_error['centers'].append(dist/px_per_mm)

    for radius_lbl, radius_res in zip(tomato_lbl['radii'], tomato_res['radii']):
        dist = abs(radius_lbl - radius_res)
        tomato_error['radii'].append(dist/px_per_mm)
  
    error_false = [None] * false_pos    
    tomato_error['centers'].extend(error_false)
    tomato_error['radii'].extend(error_false)
    
    # plot
    img_tom_res = plot_features(img_res, tomato = tomato_res)
    plot_error(img_tom_res, 
               centers = tomato_res['centers'], 
               error_centers = tomato_error['centers'], 
               error_radii =  tomato_error['radii'], 
               com_center = tomato_res['com'][0],
               com_error = tomato_error['com'],
               pwd = pwd_store, 
               name=truss_name + '_tom_error',
               use_mm = use_mm)
    
    # store
    tomato_error_all[truss_name] = tomato_error

    
    i_true, i_false = index_true_positives(peduncle_lbl['junctions'], peduncle_res['junctions'])
    
    junctions_exist = np.array(peduncle_res['junctions'])[i_true].tolist()
    junctions_fasle = np.array(peduncle_res['junctions'])[i_false].tolist()
    
    peduncle_res['junctions'] = junctions_exist
    peduncle_res['junctions'].extend(junctions_fasle)
    junctions_res = peduncle_res['junctions']  

    true_pos = len(i_true)
    false_pos = len(i_false)
    labeled_pos = len(peduncle_lbl['junctions'])
    predict_pos = len(peduncle_res['junctions'])    
    
    junctions_error = {'centers': [], 'true_pos' : true_pos, 'false_pos': false_pos, 'labeled_pos': labeled_pos, 'predict_pos': predict_pos}
    for center_lbl, center_res in zip(peduncle_lbl['junctions'], peduncle_res['junctions']):
        dist = euclidean_dist(center_lbl, center_res)
        junctions_error['centers'].append(dist/px_per_mm)

    error_false = [None] * len(junctions_fasle)    
    junctions_error['centers'].extend(error_false)
    
    img_penduncle_res = plot_features(img_res, peduncle = peduncle_res)
    plot_error(img_penduncle_res, peduncle_res['junctions'], 
               junctions_error['centers'], 
               pwd = pwd_store, 
               name=truss_name + '_pend_error',
               use_mm = use_mm)

    # store
    junction_error_all[truss_name] = junctions_error

tomato_error_centers = []
tomato_error_radii = []
tomato_error_com= []
n_true_pos = 0
n_false_pos = 0
n_labeled_pos = 0
n_predict_pos = 0

# not in order be default!
all_keys = junction_error_all.keys()
all_keys.sort()
for key in all_keys:
    print(truss_name)
    tomato_error_centers.extend(tomato_error_all[key]['centers'])
    tomato_error_radii.extend(tomato_error_all[key]['radii'])
    tomato_error_com.append(tomato_error_all[key]['com'])
    n_true_pos += tomato_error_all[key]['true_pos']
    n_false_pos += tomato_error_all[key]['false_pos']
    n_labeled_pos += tomato_error_all[key]['labeled_pos']
    n_predict_pos += tomato_error_all[key]['predict_pos']    
    
error_tomato_center_mean = np.mean(tomato_error_centers)
error_tomato_center_std = np.std(tomato_error_centers)

error_tomato_radius_mean = np.mean(tomato_error_radii)
error_tomato_radius_std = np.std(tomato_error_radii)

error_com_center_mean = np.mean(tomato_error_com)
error_com_center_std = np.std(tomato_error_com)

true_pos_perc = int(round(float(n_true_pos)/float(n_labeled_pos) * 100))
false_pos_perc = int(round(float(n_false_pos)/float(n_predict_pos) * 100))

print 'Tomato center error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_tomato_center_mean, std= error_tomato_center_std, n = n_labeled_pos, u = unit)
print 'Tomato radius error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_tomato_radius_mean, std= error_tomato_radius_std, n = n_labeled_pos, u = unit)
print 'com error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_com_center_mean, std= error_com_center_std, n = n_labeled_pos, u = unit)


print 'True positive: {true_pos:d} out of {n_tomatoes:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos, n_tomatoes= n_labeled_pos, true_pos_perc = true_pos_perc)
print 'False positive: {false_pos:d} out of {n_tomatoes:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos, n_tomatoes= n_labeled_pos, false_pos_perc = false_pos_perc)

# Junctions
junction_error_centers = []
junction_error_radii = []
n_true_pos = 0
n_false_pos = 0
n_labeled_pos = 0
n_predict_pos = 0


for key in all_keys:
    junction_error_centers.extend(junction_error_all[key]['centers'])
    n_true_pos += junction_error_all[key]['true_pos']
    n_false_pos += junction_error_all[key]['false_pos']
    n_labeled_pos += junction_error_all[key]['labeled_pos']
    n_predict_pos += junction_error_all[key]['predict_pos']
    
# mean and std without None
junction_error_centers = remove_none_from_list(junction_error_centers)
error_juncion_center_mean = np.mean(junction_error_centers)
error_junction_center_std = np.std(junction_error_centers)


true_pos_perc = int(round(float(n_true_pos)/float(n_labeled_pos) * 100))
false_pos_perc = int(round(float(n_false_pos)/float(n_predict_pos) * 100))

print 'Junction center error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_juncion_center_mean, std= error_junction_center_std, n = n_labeled_pos, u = unit)

print 'True positive: {true_pos:d} out of {n_junc_actual:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos, n_junc_actual= n_labeled_pos, true_pos_perc = true_pos_perc)
print 'False positive: {false_pos:d} out of {n_junct_predict:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos, n_junct_predict= n_predict_pos, false_pos_perc = false_pos_perc)
