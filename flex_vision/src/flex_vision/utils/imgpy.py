#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt

from skimage.measure import label, regionprops
from skimage.transform import rotate as ski_rotate

import warnings


def check_dimensions(image1, image2):
    """check_dimensions"""
    return image1.shape == image2.shape

def add(image1, image2):
    """add two images"""
    if check_dimensions(image1, image2):
        return cv2.bitwise_or(image1, image2)
    else:
        warnings.warn("Cannot add images: its dimensions do not match!")
        return None


def rotate(image, angle):
    """returns a new image, rotated by angle in radians
    angle: counter clockwise rotation in radians
    """
    dtype = image.dtype
    value_max = np.iinfo(dtype).max
    image_new = image.copy()

    # rotate returns a float in range [0, 1], this needs to be converted
    image_rotate = ski_rotate(image_new, np.rad2deg(angle), resize=True)
    image_rotate = (value_max * image_rotate).astype(dtype, copy=False)
    return image_rotate

def crop(image, angle, bbox):
    """returns a new image, rotated by angle in radians and cropped by the boundingbox"""
    image = rotate(image, angle)
    image = cut(image, bbox)
    return image

def cut(image, bbox):
    """returns the image cut, cut at the boundingbox"""
    x = bbox[0]
    y = bbox[1]
    w = bbox[2]
    h = bbox[3]
    return image[y:y+h, x:x+w]


def compute_angle(image):
    """returns the angle in radians based on the image"""
    regions = regionprops(label(image), coordinates='xy')

    if len(regions) > 1:
        warnings.warn("Multiple regions found!")

    return regions[0].orientation

def bbox(image):
    """find bounding box around a mask"""
    return cv2.boundingRect(image)