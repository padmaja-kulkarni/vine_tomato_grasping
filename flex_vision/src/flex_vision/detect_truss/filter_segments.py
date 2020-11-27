#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 17:05:38 2020

@author: taeke
"""

## imports ##
import cv2
from flex_vision.utils import remove_blobs

def filter_segments(tomato, peduncle, background, settings=None):
    # tomato
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterTom, filterDiameterTom))
    # tomato_filtered = tomato # cv2.morphologyEx(cv2.morphologyEx(tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)    

    if settings is None:
        settings = settings.filter_segments()

    # remove blobs truss
    truss = cv2.bitwise_or(tomato, peduncle)
    truss_filtered = remove_blobs(truss)
    peduncle_filtered = cv2.bitwise_and(truss_filtered, peduncle)
    tomato_filtered = cv2.bitwise_and(truss_filtered, tomato)
    
    # peduncle
    filter_diameter_pend = settings['filter_diameter']
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filter_diameter_pend, filter_diameter_pend))
    peduncle_filtered = cv2.morphologyEx(cv2.morphologyEx(peduncle_filtered, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
    
    # remove blobs peduncle
    peduncle_filtered = remove_blobs(peduncle_filtered)
    tomato_filtered = cv2.bitwise_and(truss_filtered, cv2.bitwise_not(peduncle_filtered))
    truss_filtered = cv2.bitwise_or(tomato_filtered, peduncle_filtered)
    background_filtered = cv2.bitwise_not(truss_filtered)
    
    return tomato_filtered, peduncle_filtered, background_filtered
