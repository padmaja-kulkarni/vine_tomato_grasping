#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: taeke
"""

## imports ##
import cv2
import numpy as np

# custom functions
from flex_vision.utils import plot_features

def compute_com(centers, radii):
    """
    Calculate the com of a set of spheres given their 2D centers and radii
    """
    centers = np.matrix(centers)
    radii = np.array(radii)
    return np.array((radii ** 3) * centers / (np.sum(radii ** 3)))


def detect_tomato(img_segment, settings=None, px_per_mm=None, img_rgb=None,
                  save=False, pwd="", name=""):

    if img_rgb is None:
        img_rgb = img_segment

    if settings is None:
        settings = settings.detect_tomato()

    # set dimensions
    if px_per_mm:
        radius_min_px = int(round(px_per_mm * settings['radius_min_mm']))
        radius_max_px = int(round(px_per_mm * settings['radius_max_mm']))
        distance_min_px = radius_min_px * 2
    else:
        dim = img_segment.shape
        radius_min_px = dim[1] / settings['radius_min_frac']
        radius_max_px = dim[1] / settings['radius_max_frac']
        distance_min_px = radius_min_px * 2

    # Hough requires a gradient, thus the image is blurred
    blur_size = settings['blur_size']
    truss_blurred = cv2.GaussianBlur(img_segment, (blur_size, blur_size), 0)

    # intialize
    centers_overlap = radii_overlap = com_overlap = None
    centers = radii = com = None


    # fit circles: [x, y, radius]
    circles = cv2.HoughCircles(truss_blurred,
                               cv2.HOUGH_GRADIENT,
                               settings['dp'],
                               distance_min_px,
                               param1=settings['param1'],
                               param2=settings['param2'],
                               minRadius=radius_min_px,
                               maxRadius=radius_max_px)

    if circles is not None:

        centers_overlap = circles[0][:, 0:2] # [x, y, r]
        radii_overlap = circles[0][:, 2]
        com_overlap = compute_com(centers_overlap, radii_overlap)
        n_overlap = len(radii_overlap)

        # remove circles which do not overlapp with the tomato segment
        i_keep = find_overlapping_tomatoes(centers_overlap,
                                           radii_overlap,
                                           img_segment,
                                           ratio_threshold=settings['ratio_threshold'])

        n = len(i_keep)
        if n != n_overlap:
            print 'removed', n_overlap - n, 'tomaoto(es) based on overlap'

        if len(i_keep) != 0:
            centers = centers_overlap[i_keep, :]
            radii = radii_overlap[i_keep]
            com = compute_com(centers, radii)

    # visualize result
    if save:
        tomato = {'centers': centers, 'radii': radii, 'com': com}
        tomato_overlap = {'centers': centers_overlap, 'radii': radii_overlap, 'com': com_overlap}
        plot_features(img_rgb, tomato=tomato_overlap, pwd=pwd, file_name=name + '_1', zoom=True)
        plot_features(img_rgb, tomato=tomato, pwd=pwd, file_name=name + '_2', zoom=True)

    return centers, radii, com


def find_overlapping_tomatoes(centers, radii, img_segment, ratio_threshold=0.5):
    iKeep = []
    N = centers.shape[0]
    for i in range(0, N):

        image_empty = np.zeros(img_segment.shape, dtype=np.uint8)
        mask = cv2.circle(image_empty, (centers[i, 0], centers[i, 1]), radii[i], 255, -1)

        res = cv2.bitwise_and(img_segment, mask)
        pixels = np.sum(res == 255)
        total = np.pi * radii[i] ** 2
        ratio = pixels / total
        if ratio > ratio_threshold:
            iKeep.append(i)

    return iKeep


if __name__ == '__main__':
    print("This file has no main!")
