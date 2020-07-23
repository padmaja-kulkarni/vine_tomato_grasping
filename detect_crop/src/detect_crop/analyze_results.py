# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:22:48 2020

@author: taeke
"""


import os
import json
import numpy as np
from util import load_rgb, plot_features, plot_error, make_dirs, change_brightness

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
i_end = 22
N = i_end - i_start + 1

pwd_current = os.path.dirname(__file__)
dataset ='real_blue' # "drawing" #  "empty" # "artificial" # 

pwd_lbl = os.path.join(pwd_current, "..", "data", dataset)
pwd_res = os.path.join(pwd_current, "..", "results", dataset, 'json')
pwd_store = os.path.join(pwd_current, "..", "results", dataset, 'final')

make_dirs(pwd_store)

tomato_error_all = {}
junction_error_all = {}

for count, i_tomato in enumerate(range(i_start, i_end + 1)):
    print("Analyzing image %d out of %d" %(count + 1, N))

    truss_name = str(i_tomato).zfill(3)
    
    file_lbl = os.path.join(pwd_lbl, truss_name + '.json')
    file_res = os.path.join(pwd_res, truss_name + '.json')
    
    # load data
    img_rgb = load_rgb(pwd_lbl, truss_name + '.png', horizontal = False)
    
    if not os.path.exists(file_lbl):
        print('JSON does not exist for tomato: ' + truss_name)
        continue
    
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
           
    #img_rgb = change_brightness(img_rgb.copy(), -0.9) # change_brightness( , 0.8)
    img_lbl = img_rgb.copy()
    plot_features(img_lbl, tomato_lbl, peduncle_lbl, [], pwd = pwd_store, file_name=truss_name + '_lbl')
    
    with open(file_res, "r") as read_file:
        data_results = json.load(read_file)   
        
    img_res = img_rgb.copy()   
    tomato_res = data_results['tomato']
    peduncle_res = data_results['peduncle']
    
    # TODO: false negative!
    n_tomato = len(tomato_res['centers'])
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
    
    # compute error
    tomato_error = {'radii': [], 'centers': [], 'true_pos': true_pos, 'false_pos': false_pos, 'n_tomato': n_tomato}
    for center_lbl, center_res in zip(tomato_lbl['centers'], tomato_res['centers']):
        dist = euclidean_dist(center_lbl, center_res)
        tomato_error['centers'].append(dist)

    for radius_lbl, radius_res in zip(tomato_lbl['radii'], tomato_res['radii']):
        dist = abs(radius_lbl - radius_res)
        tomato_error['radii'].append(dist)
  
    error_false = [None] * false_pos    
    tomato_error['centers'].extend(error_false)
    tomato_error['radii'].extend(error_false)
      
    # plot
    img_res = plot_features(img_res, tomato_res, peduncle_res, [])
    plot_error(img_res.copy(), 
               centers = tomato_res['centers'], 
               error_centers = tomato_error['centers'], 
               error_radii =  tomato_error['radii'], 
               pwd = pwd_store, 
               name=truss_name + '_tom_error')
    
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
    n_junction = len(peduncle_lbl['junctions'])
    
    junctions_error = {'centers': [], 'true_pos' : true_pos, 'false_pos': false_pos, 'n_junction': n_junction}
    for center_lbl, center_res in zip(peduncle_lbl['junctions'], peduncle_res['junctions']):
        dist = euclidean_dist(center_lbl, center_res)
        junctions_error['centers'].append(dist)

    error_false = [None] * len(junctions_fasle)    
    junctions_error['centers'].extend(error_false)

    plot_error(img_res.copy(), peduncle_res['junctions'], 
               junctions_error['centers'], 
               pwd = pwd_store, 
               name=truss_name + '_pend_error')

    # store
    junction_error_all[truss_name] = junctions_error

tomato_error_centers = []
tomato_error_radii = []
n_true_pos = 0
n_false_pos = 0
n_tomatoes = 0

for truss_name in tomato_error_all:
    tomato_error_centers.extend(tomato_error_all[truss_name]['centers'])
    tomato_error_radii.extend(tomato_error_all[truss_name]['radii'])
    n_true_pos += tomato_error_all[truss_name]['true_pos']
    n_false_pos += tomato_error_all[truss_name]['false_pos']
    n_tomatoes += tomato_error_all[truss_name]['n_tomato']
    
    
error_tomato_center_mean = np.mean(tomato_error_centers)
error_tomato_center_std = np.std(tomato_error_centers)

error_tomato_radius_mean = np.mean(tomato_error_radii)
error_tomato_radius_std = np.std(tomato_error_radii)

true_pos_perc = int(round(float(n_true_pos)/float(n_tomatoes) * 100))
false_pos_perc = int(round(float(n_false_pos)/float(n_tomatoes) * 100))

print 'Tomato center error: {mean:.2f} px +- {std:.2f} px (n = {n:d})'.format(mean=error_tomato_center_mean, std= error_tomato_center_std, n = n_tomatoes)
print 'Tomato radius error: {mean:.2f} px +- {std:.2f} px (n = {n:d})'.format(mean=error_tomato_radius_mean, std= error_tomato_radius_std, n = n_tomatoes)

print 'True positive: {true_pos:d} out of {n_tomatoes:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos, n_tomatoes= n_tomatoes, true_pos_perc = true_pos_perc)
print 'False positive: {false_pos:d} out of {n_tomatoes:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos, n_tomatoes= n_tomatoes, false_pos_perc = false_pos_perc)

# Junctions
junction_error_centers = []
junction_error_radii = []
n_true_pos = 0
n_false_pos = 0
n_junctions = 0

for truss_name in junction_error_all:
    junction_error_centers.extend(junction_error_all[truss_name]['centers'])
    n_true_pos += junction_error_all[truss_name]['true_pos']
    n_false_pos += junction_error_all[truss_name]['false_pos']
    n_junctions += junction_error_all[truss_name]['n_junction']
    
# TODO: std and mean not nan
error_juncion_center_mean = np.mean(junction_error_centers)
error_junction_center_std = np.std(junction_error_centers)


true_pos_perc = int(round(float(n_true_pos)/float(n_junctions) * 100))
false_pos_perc = int(round(float(n_false_pos)/float(n_junctions) * 100))

print 'Junction center error: {mean:.2f} px +- {std:.2f} px (n = {n:d})'.format(mean=error_juncion_center_mean, std= error_junction_center_std, n = n_tomatoes)

print 'True positive: {true_pos:d} out of {n_tomatoes:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos, n_tomatoes= n_junctions, true_pos_perc = true_pos_perc)
print 'False positive: {false_pos:d} out of {n_tomatoes:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos, n_tomatoes= n_junctions, false_pos_perc = false_pos_perc)
