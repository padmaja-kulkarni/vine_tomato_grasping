# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:22:48 2020

@author: taeke
"""


import os
import json

from util import load_rgb, add_circles
from matplotlib import pyplot as plt

def euclidean_dist(p1, p2):
    'compute the euclidean distance between point_1 and point_2'
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

i_tomato = 1
tomato_name = str(i_tomato).zfill(3)

label_png = 'apc2016_obj3_json/label.png'

pwd_current = os.path.dirname(__file__)
dataset ='real_blue' # "drawing" #  "empty" # "artificial" # 

pwd_data = os.path.join(pwd_current, "..", "data", dataset)
pwd_results = os.path.join(pwd_current, "..", "results", dataset)

file_lbl = os.path.join(pwd_data, tomato_name + '.json')
file_res = os.path.join(pwd_results, tomato_name + '.json')

img_pwd = os.path.join(pwd_data, )

# load data
img_rgb = load_rgb(pwd_data, tomato_name + '.png', horizontal = False)
with open(file_lbl, "r") as read_file:
    data_lbl = json.load(read_file) 
    
data_lbl.pop('imageData', None)
shapes = data_lbl['shapes']

tomato_lbl = {'radii': [], 'centers': []}
nodes_lbl = {'junctions': [], 'end_points': []}

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
        nodes_lbl['junctions'].append(point)

    elif label == 'end_point':
        if shape_type != 'point':
            print("I do not know what to do with ", label, " of shape type ", shape_type)
            
        point = shape['points'][0]
        nodes_lbl['end_points'].append(point)
        
    else:
        print "i do not know what to do with ", label
        
img_lbl = img_rgb.copy()        
img_lbl = add_circles(img_lbl, tomato_lbl['centers'], radii = tomato_lbl['radii'])
img_lbl = add_circles(img_lbl, nodes_lbl['junctions'])
img_lbl = add_circles(img_lbl, nodes_lbl['end_points'])
plt.imshow(img_lbl)

with open(file_res, "r") as read_file:
    data_results = json.load(read_file)   
    
img_res = img_rgb.copy()   
tomato_res = data_results['tomato']
img_res = add_circles(img_res, tomato_res['centers'], radii = tomato_res['radii'])

plt.imshow(img_res)