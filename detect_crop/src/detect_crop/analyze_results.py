# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:22:48 2020

@author: taeke
"""


import os
import json
import numpy as np
from util import load_rgb, plot_features, plot_error, make_dirs

def euclidean_dist(p1, p2):
    'compute the euclidean distance between point_1 and point_2'
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

def euclidean_dist_list(p1, lst):
    dist = []
    for p2 in lst:
        dist.append(euclidean_dist(p1, p2))
        
    return dist
    
    
def order_centers_as_key(centers, centers_key):
    
    i = []
    for center in centers:
        dist = euclidean_dist_list(center, centers_key)
        i.append(np.argmin(dist))
        
    return i

i_start = 1
i_end = 22
N = i_end - i_start + 1

pwd_current = os.path.dirname(__file__)
dataset ='real_blue' # "drawing" #  "empty" # "artificial" # 

pwd_lbl = os.path.join(pwd_current, "..", "data", dataset)
pwd_res = os.path.join(pwd_current, "..", "results", dataset, 'json')
pwd_store = os.path.join(pwd_res, 'final')

make_dirs(pwd_store)

for count, i_tomato in enumerate(range(i_start, i_end + 1)):
    print("Analyzing image %d out of %d" %(count + 1, N))

    tomato_name = str(i_tomato).zfill(3)
    
    file_lbl = os.path.join(pwd_lbl, tomato_name + '.json')
    file_res = os.path.join(pwd_res, tomato_name + '.json')
    
    # load data
    img_rgb = load_rgb(pwd_lbl, tomato_name + '.png', horizontal = False)
    
    if not os.path.exists(file_lbl):
        print('JSON does not exist for tomato: ' + tomato_name)
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
            
    img_lbl = img_rgb.copy() # change_brightness( , 0.8)
    plot_features(img_lbl, tomato_lbl, peduncle_lbl, [], pwd = pwd_store, file_name=tomato_name + '_lbl')
    
    with open(file_res, "r") as read_file:
        data_results = json.load(read_file)   
        
    img_res = img_rgb.copy()   
    tomato_res = data_results['tomato']
    peduncle_res = data_results['peduncle']
    
    i = order_centers_as_key(tomato_res['centers'], tomato_lbl['centers'])
    tomato_res['centers'] = [x for _,x in sorted(zip(i,tomato_res['centers']))] 
    tomato_res['radii'] = [x for _,x in sorted(zip(i,tomato_res['radii']))] 
        

    error = {'radii': [], 'centers': []}
    for center_lbl, center_res in zip(tomato_lbl['centers'], tomato_res['centers']):
        dist = euclidean_dist(center_lbl, center_res)
        error['centers'].append(dist)

    for radius_lbl, radius_res in zip(tomato_lbl['radii'], tomato_res['radii']):
        dist = abs(radius_lbl - radius_res)
        error['radii'].append(dist)
        
    img_res = plot_features(img_res, tomato_res, peduncle_res, [])
    centers = tomato_res['centers']
    error_centers = error['centers']
    error_radii = error['radii']
    plot_error(img_res.copy(), centers, error_centers, error_radii = error_radii, 
               pwd = pwd_store, name=tomato_name + 'tom_error', title = 'Tomato Error')
    

    i_true = []
    for i_lbl ,junction_lbl in enumerate(peduncle_lbl['junctions']):    
        dist = euclidean_dist_list(junction_lbl, peduncle_res['junctions']  )
        i_res = np.argmin(dist)
        # print(i_res)
        # print 'the', i_res, 'th found junction, should have label', i_lbl 
        # print()
        i_true.append(i_res)
            
    i_false = list(set(range(len(peduncle_res['junctions']))) -  set(i_true))    
    
    
    junctions_exist = np.array(peduncle_res['junctions'])[i_true].tolist()
    junctions_fasle = np.array(peduncle_res['junctions'])[i_false].tolist()
    
    
    peduncle_res['junctions'] = junctions_exist
    peduncle_res['junctions'].extend(junctions_fasle)
    junctions_res = peduncle_res['junctions']  

    error = {'centers': []}
    for center_lbl, center_res in zip(peduncle_lbl['junctions'], peduncle_res['junctions']):
        dist = euclidean_dist(center_lbl, center_res)
        error['centers'].append(dist)

    error_false = [None] * len(junctions_fasle)    
    error['centers'].extend(error_false)
    centers = peduncle_res['junctions']
    error_centers = error['centers']
    plot_error(img_res.copy(), centers, error_centers, pwd = pwd_store, 
               name=tomato_name + '_pend_error', title = 'Peduncle Error')