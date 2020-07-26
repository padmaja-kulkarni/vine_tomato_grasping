#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 17:05:38 2020

@author: taeke
"""

## imports ##
import cv2
from util import remove_blobs

def filter_segments(tomato, peduncle, background):
    # tomato
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterTom, filterDiameterTom))
    # tomatoFiltered = tomato # cv2.morphologyEx(cv2.morphologyEx(tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)    
    
    # remove blobs truss
    truss = cv2.bitwise_or(tomato, peduncle)
    trussFiltered = remove_blobs(truss)
    peduncleFiltered = cv2.bitwise_and(trussFiltered, peduncle)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, tomato)
    
    # peduncle
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (filterDiameterPend, filterDiameterPend))
    # peduncleFiltered = cv2.morphologyEx(cv2.morphologyEx(peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)    
    
    # remove blobs peduncle
    peduncleFiltered = remove_blobs(peduncleFiltered)
    tomatoFiltered = cv2.bitwise_and(trussFiltered, cv2.bitwise_not(peduncleFiltered))
    trussFiltered = cv2.bitwise_or(tomatoFiltered, peduncleFiltered)
    backgroundFiltered = cv2.bitwise_not(truss)    
    
    return tomatoFiltered, peduncleFiltered, backgroundFiltered