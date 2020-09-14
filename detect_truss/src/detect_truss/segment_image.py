# -*- coding: utf-8 -*-
"""
Created on Wed May 20 13:32:31 2020

@author: taeke
"""

## imports ##
import copy
import cv2
import numpy as np

from matplotlib import pyplot as plt
import matplotlib as mpl
from sklearn.metrics.pairwise import euclidean_distances

# custom functions
from util import bin2img
from util import save_fig
from util import angular_difference
from util import plot_segments

final_image_id = '015'

def k_means_hue(img_hue, n_clusters, centers=None):
    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2 * np.float32(img_hue.flatten()))
    data = np.stack((np.cos(angle), np.sin(angle)), axis=1)

    if centers is not None:
        labels = np.array(assign_labels(img_hue, centers)[:, 0], dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS  # + cv2.KMEANS_PP_CENTERS
    else:
        labels = None
        flags = cv2.KMEANS_PP_CENTERS

    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 10, np.sin(np.deg2rad(1.0)))
    compactness, labels, centers_xy = cv2.kmeans(data=data,
                                                 K=n_clusters,
                                                 bestLabels=labels,  # None, #
                                                 criteria=criteria,
                                                 attempts=1,
                                                 flags=flags)  # cv2.KMEANS_PP_CENTERS) #

    # convert the centers from xy to angles\
    centers_out = {}
    centers_out['hue'] = np.arctan2(centers_xy[:, 1], centers_xy[:, 0])
    return centers_out, labels


def k_means_hue_a(img_hue, img_a, n_clusters, centers=None):
    # convert hue value to angles, and place on unit circle
    angle = np.deg2rad(2 * np.float32(img_hue.flatten()))

    data = np.stack((np.cos(angle), np.sin(angle), img_a.flatten()), axis=1)

    if centers is not None:
        labels = np.array(assign_labels(img_hue, centers), dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS  # + cv2.KMEANS_PP_CENTERS
    else:
        labels = None
        flags = cv2.KMEANS_PP_CENTERS

    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS, 20, np.sin(np.deg2rad(1.0)))
    compactness, labels, centers_xy = cv2.kmeans(data=data,
                                                 K=n_clusters,
                                                 bestLabels=labels,  # None, #
                                                 criteria=criteria,
                                                 attempts=1,
                                                 flags=flags)  # cv2.KMEANS_PP_CENTERS) #

    # convert the centers from xy to angles\
    centers_out = {}
    centers_out['hue'] = np.arctan2(centers_xy[:, 1], centers_xy[:, 0])
    centers_out['a'] = centers_xy[:, 2]
    return centers_out, labels


def normalize_image(img, scale=1.0):
    val_min = np.min(img)
    val_max = np.max(img)
    img_norm = np.float32(img - val_min) / np.float32(val_max - val_min) * scale
    return img_norm


def assign_labels(img_hue, centers_dict, a_weight=1.0, img_a=None):
    data_angle = np.deg2rad(2 * np.float32(img_hue.flatten()))  # [rad]
    data = np.stack((np.cos(data_angle), np.sin(data_angle)), axis=1)
    center_angle = centers_dict['hue']
    centers = np.stack((np.cos(center_angle), np.sin(center_angle)), axis=1)

    if img_a is not None:
        data_a = np.expand_dims(normalize_image(img_a, scale=a_weight).flatten(), axis=1)
        data = np.append(data, data_a, axis=1)
        center_a = np.expand_dims(centers_dict['a'], axis=1)
        centers = np.append(centers, center_a, axis=1)

    dist = euclidean_distances(data, centers)
    labels = np.argmin(dist, axis=1)
    return labels


def segment_truss(img_hue, img_a=None, save="False", name="", pwd=""):
    a_weight = 1
    n = 3
    centers_prior = {'hue': {}}
    centers = {}
    centers_prior['hue']['tomato'] = np.deg2rad(0)  # [rad]
    centers_prior['hue']['peduncle'] = np.deg2rad(90)
    centers_prior['hue']['background'] = np.deg2rad(240)  # [rad]
    centers['hue'] = centers_prior['hue'].values()

    if img_a is None:
        centers, labels = k_means_hue(img_hue, n, centers=centers)  # centers
    else:
        a_min = np.min(img_a)
        a_max = np.max(img_a)
        img_a_norm = normalize_image(img_a, scale=a_weight)

        centers, labels = k_means_hue_a(img_hue, img_a_norm, n, centers=centers)  # centers

    # determine which center corresponds to which segment
    lbl = {}
    lbl["tomato"] = np.argmin(angular_difference(centers['hue'], centers_prior['hue']['tomato']))
    lbl["background"] = np.argmin(angular_difference(centers['hue'], centers_prior['hue']['background']))
    lbl["peduncle"] = list(set(range(0, n)) - set(lbl.values()))[0]

    # compute masks
    dim = img_hue.shape
    tomato = label2img(labels, lbl["tomato"], dim)
    if np.abs(centers['hue'][lbl["background"]] - centers['hue'][lbl["peduncle"]]) < np.deg2rad(10):
        print "did not detect a peduncle"
        peduncle = bin2img(np.zeros(dim))
        background = cv2.bitwise_not(tomato)
    else:
        peduncle = label2img(labels, lbl["peduncle"], dim)
        background = label2img(labels, lbl["background"], dim)
    if save:
        both_hist(img_hue, img_a_norm, centers, lbl, a_bins=a_max-a_min+1, pwd=pwd, name=name)
        hue_hist(img_hue, np.rad2deg(centers['hue']), lbl, name, pwd)
        if img_a is not None:
            a_hist(img_a_norm, centers['a'], lbl, bins=a_max-a_min+1, name=name, pwd=pwd)

    return background, tomato, peduncle


def hue_hist(img_hue, centers, lbl, name, pwd):
    # [-180, 180] => [0, 360]
    centers[centers < 0] += 360

    # plot Hue (HSV)
    fig, ax = plt.subplots(1)
    plt.yscale("log")

    center_background = centers[lbl["background"]]
    center_tomato = centers[lbl["tomato"]]
    center_peduncle = centers[lbl["peduncle"]]

    ax.axvline(x=center_background, color='b')
    ax.axvline(x=center_tomato, color='r')
    ax.axvline(x=center_peduncle, color='g')

    x0 = 0
    x1 = (center_tomato + center_peduncle) / 2
    x2 = (center_peduncle + center_background) / 2
    x3 = (center_background + center_tomato + 360) / 2
    x4 = 360
    alpha = 0.2

    plt.axvspan(x0, x1, color='r', alpha=alpha, lw=0)
    plt.axvspan(x1, x2, color='g', alpha=alpha, lw=0)
    plt.axvspan(x2, x3, color='b', alpha=alpha, lw=0)
    plt.axvspan(x3, x4, color='r', alpha=alpha, lw=0)

    bins = 180
    angle = img_hue.flatten().astype('uint16') * 2
    radii, bins, patches = ax.hist(angle, bins=bins, range=(0, 360), color="black", lw=0)
    ax.set_xlabel("hue [$^\circ$]")
    ax.set_ylabel("frequency")
    save_fig(fig, pwd, name + "_hue_hist", titleSize=10)


def a_hist(img_a, centers, lbl, bins=80, name="", pwd=""):
    # plot Hue (HSV)
    fig, ax = plt.subplots(1)
    plt.yscale("log")

    center_background = centers[lbl["background"]]
    center_tomato = centers[lbl["tomato"]]
    center_peduncle = centers[lbl["peduncle"]]

    ax.axvline(x=center_background, color='b')
    ax.axvline(x=center_tomato, color='r')
    ax.axvline(x=center_peduncle, color='g')

    x0 = 0
    x1 = (center_background + center_peduncle) / 2
    x2 = (center_peduncle + center_tomato) / 2
    x3 = 1
    alpha = 0.2

    plt.axvspan(x0, x1, color='b', alpha=alpha, lw=0)
    plt.axvspan(x1, x2, color='g', alpha=alpha, lw=0)
    plt.axvspan(x2, x3, color='r', alpha=alpha, lw=0)


    angle = img_a.flatten()  # .astype('uint16')
    radii, bins, patches = ax.hist(angle, bins=bins, range=(0, 1), color="black", lw=0)
    ax.set_xlabel("a")
    ax.set_ylabel("frequency")
    save_fig(fig, pwd, name + "_a_hist", titleSize=10)

def both_hist(img_hue, img_a, centers, lbl, a_bins=80, pwd="", name=""):

    scale = 2
    hue_min = 0
    hue_max = 180
    hue_bins = 180*scale
    hue_step = float(hue_max)/float(hue_bins)
    a_min = np.min(img_a)
    a_max = np.max(img_a)
    a_bins = a_bins*scale
    a_step = float(a_max - a_min)/float(a_bins)

    # label every possible location
    img_a_test, img_hue_test = np.mgrid[a_min:a_max:(a_step), hue_min:hue_max:(hue_step)]
    labels = assign_labels(img_hue_test, centers, img_a=img_a_test, a_weight=1.5)
    tomato = label2img(labels, lbl["tomato"], img_hue_test.shape)
    peduncle = label2img(labels, lbl["peduncle"], img_hue_test.shape)
    background = label2img(labels, lbl["background"], img_hue_test.shape)

    #
    hue_range = [0, 360]
    a_range = [a_min, a_max]
    my_range = [a_range, hue_range]
    x = img_a.flatten()
    y = img_hue.flatten().astype('uint16') * 2

    hist, _, _, _ = plt.hist2d(x, y, range=my_range, bins=[a_bins/scale, hue_bins/scale])
    hist[hist > 0] = np.log(hist[hist > 0])
    img_hist_gray = 255 - (hist / np.amax(hist) * 255).astype(np.uint8)
    img_hist_rgb = cv2.applyColorMap(img_hist_gray, cv2.COLORMAP_JET)

    # rescale based on scala param
    new_shape = (img_hue_test.shape[1], img_hue_test.shape[0])  # [width, height]
    img_hist_rgb = cv2.resize(img_hist_rgb, new_shape,  interpolation=cv2.INTER_NEAREST)

    # overlay with histogram
    img = plot_segments(img_hist_rgb, background, tomato, peduncle, show_background=True, alpha=0.2, thickness=1, use_image_colours=False)

    # plot cluster centers
    centers_hue = np.rad2deg(centers['hue'])
    centers_hue[centers_hue < 0] += 360
    hue_coords = (centers_hue / 2) / hue_step  # [rad]
    a_coords = (centers['a']) / a_step
    fig = plt.figure()
    plt.imshow(img)

    for label in lbl:
        i = lbl[label]
        if label == 'tomato':
            color = 'r'
        elif label == 'peduncle':
            color='g'
        elif label == 'background':
            color='b'
        plt.plot(hue_coords[i], a_coords[i], 'o', color=color, markeredgecolor='w')

    # fix axis, only plot x axis for last image, only plot title for first
    if name == final_image_id:
        plt.xticks([hue_min, hue_bins / 2, hue_bins - 1], map(str, (0, 180, 360)))
        plt.xlabel("hue [$^\circ$]")
    else:
        plt.xticks([])

    plt.ylabel("a")
    plt.yticks([0, a_bins/2, a_bins-1], map(str, (0.0, 0.5, 1.0)))

    save_fig(fig, pwd, name + "_hist", no_ticks=False)

def label2img(labels, label, dim):
    data = labels.ravel() == label
    img = data.reshape(dim)
    return bin2img(img)


def hue_scatter(xy, RGB):
    rows, cols = RGB.shape[:2]

    pixel_colors = RGB.reshape((rows * cols, 3))
    norm = mpl.colors.Normalize(vmin=-1., vmax=1.)
    norm.autoscale(pixel_colors)
    pixel_colors = norm(pixel_colors).tolist()

    fig = plt.figure()
    ax = fig.add_subplot(2, 2, 1)
    ax.scatter(xy[:, 0], xy[:, 1], facecolors=pixel_colors, marker=".")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    plt.show()
