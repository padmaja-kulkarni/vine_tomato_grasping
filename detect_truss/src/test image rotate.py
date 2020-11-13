#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 17 14:35:14 2020

@author: taeke
"""


import numpy as np
from matplotlib import pyplot as plt
from src.detect_truss.transform import Transform, Point2D
from skimage.transform import rotate

# custom


ORIGINAL_FRAME_ID = 'original'
LOCAL_FRAME_ID = 'local'


def main():

    #%% INIT
    H = 400
    W = 1000
    shape = (H, W)

    row = 100
    col = 400

    coord_o = np.array([[row], [col]]) # row, col
    point = Point2D(coord_o, ORIGINAL_FRAME_ID)
    vmin = 0
    vmax = 255


    plt.close('all')

    for angle_deg in range(-90, 90, 10):
        print(angle_deg)
        angle_rad = angle_deg/180.0*np.pi

        # create image
        img = np.full(shape, 100, dtype=np.uint8)
        img[coord_o[0], coord_o[1]] = 255

        # rotate image
        transform = Transform(ORIGINAL_FRAME_ID, LOCAL_FRAME_ID, img.shape, angle_rad)

        imgR = vmax*rotate(img, angle_deg, resize=True)

        coord_r = point.get_coord(transform, LOCAL_FRAME_ID)
        point_new = Point2D(coord_r, LOCAL_FRAME_ID)
        coord_p = point_new.get_coord(transform, ORIGINAL_FRAME_ID)


        fig = plt.figure()
        plt.subplot(2, 2, 1)
        ax = fig.gca()
        plt.imshow(img, vmin=vmin, vmax=vmax)
        circle = plt.Circle((coord_o[1], coord_o[0]), 3, color='r', fill=False)
        ax.add_artist(circle)

        plt.subplot(2, 2, 2)
        ax = fig.gca()
        plt.imshow(imgR, vmin=vmin, vmax=vmax)
        circle = plt.Circle((coord_r[1], coord_r[0]), 3, color='r', fill=False)
        ax.add_artist(circle)


        plt.subplot(2, 2, 3)
        ax = fig.gca()
        plt.imshow(img, vmin=vmin, vmax=vmax)

        circle = plt.Circle((coord_p[1], coord_p[0]), 3, color='r', fill=False)
        ax.add_artist(circle)


        plt.show()

if __name__ == '__main__':
    main()