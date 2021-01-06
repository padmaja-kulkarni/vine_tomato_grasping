# -*- coding: utf-8 -*-
"""
Created on Wed May 20 13:32:31 2020

@author: taeke
"""

## imports ##
import cv2
import numpy as np

from matplotlib import pyplot as plt
import matplotlib as mpl

from sklearn.metrics.pairwise import euclidean_distances

# custom functions
from flex_vision.utils.util import bin2img
from flex_vision.utils.util import save_fig
from flex_vision.utils.util import angular_difference
from flex_vision.utils.util import plot_segments
from flex_vision.utils.util import grey_2_rgb

final_image_id = '015' # used for plotting


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


# def to_dataset_3d(img_hue, img_a, a_weight=1):
#     angle = np.deg2rad(2 * np.float32(img_hue.flatten()))
#     data = np.stack((np.cos(angle), np.sin(angle), img_a.flatten()), axis=1)

def k_means_hue_a(img_hue, img_a, n_clusters, settings, centers=None):
    # convert hue value to angles, and place on unit circle

    h, w = img_hue.shape

    new_shape = (w / settings['f'], h / settings['f'])
    img_hue = cv2.resize(img_hue, new_shape, interpolation=cv2.INTER_NEAREST)
    img_a = cv2.resize(img_a, new_shape, interpolation=cv2.INTER_NEAREST)

    angle = np.deg2rad(2 * np.float32(img_hue.flatten()))
    hue_radius = settings['hue_radius']
    data = np.stack((hue_radius * np.cos(angle), hue_radius * np.sin(angle), img_a.flatten()), axis=1)

    if centers is not None:
        labels = np.array(assign_labels(img_hue, centers, img_a=img_a), dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS
        attempts = 1
    else:
        labels = None
        flags = cv2.KMEANS_PP_CENTERS
        attempts = 3

    # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, settings['i_max'], settings['epsilon'])
    compactness, labels, centers_xy = cv2.kmeans(data=data,
                                                 K=n_clusters,
                                                 bestLabels=labels,  # None, #
                                                 criteria=criteria,
                                                 attempts=attempts,
                                                 flags=flags)  # cv2.KMEANS_PP_CENTERS) #

    # convert the centers from xy to angles\
    centers_out = {}
    centers_out['hue'] = np.arctan2(centers_xy[:, 1], centers_xy[:, 0])
    centers_out['a'] = centers_xy[:, 2]
    return centers_out, labels



def assign_labels(img_hue, centers_dict, hue_radius=1.0, img_a=None):
    data_angle = np.deg2rad(2 * np.float32(img_hue.flatten()))  # [rad]
    data = np.stack((hue_radius * np.cos(data_angle), hue_radius * np.sin(data_angle)), axis=1)
    center_angle = centers_dict['hue']
    centers = np.stack((hue_radius * np.cos(center_angle), hue_radius * np.sin(center_angle)), axis=1)

    if img_a is not None:
        data_a = np.expand_dims(normalize_image(img_a).flatten(), axis=1)
        data = np.append(data, data_a, axis=1)
        center_a = np.expand_dims(centers_dict['a'], axis=1)
        centers = np.append(centers, center_a, axis=1)

    dist = euclidean_distances(data, centers)
    labels = np.argmin(dist, axis=1)
    return labels

def normalize_image(img):
    """Normalizes image into the interval [-1, 1]"""
    val_min = np.min(img)
    val_max = np.max(img)
    img_norm = np.float32(img - val_min) / np.float32(val_max - val_min) * 2 - 1
    return img_norm

def segment_truss(img_hue, img_a=None, save="False", name="", pwd="", settings=None):
    if settings is None:
        settings = settings.segment_image()

    n = 3

    centers_prior_hue = {'tomato': np.deg2rad(0),  # [rad]
                         'peduncle': np.deg2rad(90),  # [rad]
                         'background': np.deg2rad(240)}  # [rad]
    centers_prior_a = {'tomato': 1.0,
                       'peduncle': 0.5,
                       'background': 0.0}

    centers_prior = {'hue': centers_prior_hue, 'a': centers_prior_a}
    centers = {'hue': centers_prior['hue'].values(), 'a': centers_prior['a'].values()}

    if img_a is None:
        centers, labels = k_means_hue(img_hue, n, centers=centers)  # centers
    else:
        a_min = np.min(img_a)
        a_max = np.max(img_a)
        img_a_norm = normalize_image(img_a)
        centers, _ = k_means_hue_a(img_hue, img_a_norm, n, settings, centers=centers)  # centers
        labels = assign_labels(img_hue, centers, hue_radius=settings['hue_radius'], img_a=img_a)

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
        both_hist(img_hue, img_a_norm, centers, lbl, a_bins=a_max - a_min + 1, pwd=pwd, name=name,
                  hue_radius=settings['hue_radius'])
        hue_hist(img_hue, np.rad2deg(centers['hue']), lbl, name, pwd)
        if img_a is not None:
            a_hist(img_a_norm, centers['a'], lbl, bins=a_max - a_min + 1, name=name, pwd=pwd)

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


def a_hist(img_a, centers, lbl, bins=80, a_min=-1.0, a_max=1.0, name="", pwd=""):
    # plot Hue (HSV)
    fig, ax = plt.subplots(1)
    plt.yscale("log")

    center_background = centers[lbl["background"]]
    center_tomato = centers[lbl["tomato"]]
    center_peduncle = centers[lbl["peduncle"]]

    ax.axvline(x=center_background, color='b')
    ax.axvline(x=center_tomato, color='r')
    ax.axvline(x=center_peduncle, color='g')

    x0 = a_min
    x1 = (center_background + center_peduncle) / 2
    x2 = (center_peduncle + center_tomato) / 2
    x3 = a_max
    alpha = 0.2

    plt.axvspan(x0, x1, color='b', alpha=alpha, lw=0)
    plt.axvspan(x1, x2, color='g', alpha=alpha, lw=0)
    plt.axvspan(x2, x3, color='r', alpha=alpha, lw=0)

    angle = img_a.flatten()  # .astype('uint16')
    radii, bins, patches = ax.hist(angle, bins=bins, range=(a_min, a_max), color="black", lw=0)
    ax.set_xlabel("a")
    ax.set_ylabel("frequency")
    save_fig(fig, pwd, name + "_a_hist", titleSize=10)


def both_hist(img_hue, img_a, centers, lbl, a_bins=80, pwd="", name="", hue_min=0, hue_max=180, hue_radius=1.0,
              a_min=-1.0, a_max=1.0, true_scale=False):
    a_height = 500
    if true_scale:
        hue_height = 2 * np.pi * hue_radius / 2 * a_height  # 180*scale
    else:
        hue_height = 2 * 500

    hue_step = float(hue_max - hue_min) / float(hue_height)
    a_step = float(a_max - a_min) / float(a_height)

    # label every possible location
    values_a, values_hue = np.mgrid[a_min:a_max:a_step, hue_min:hue_max:hue_step]
    shape = values_hue.shape
    labels = assign_labels(values_hue, centers, img_a=values_a, hue_radius=hue_radius)
    tomato = label2img(labels, lbl["tomato"], shape)
    peduncle = label2img(labels, lbl["peduncle"], shape)
    background = label2img(labels, lbl["background"], shape)

    #
    hue_range = [0, 360]
    a_range = [a_min, a_max]
    my_range = [a_range, hue_range]
    x = img_a.flatten()
    y = img_hue.flatten().astype('uint16') * 2

    scale = 4.0
    hue_bins = 180 * scale
    a_bins = a_bins * scale

    hist, _, _, _ = plt.hist2d(x, y, range=my_range, bins=[a_bins / scale, hue_bins / scale])
    hist[hist > 0] = np.log(hist[hist > 0])
    img_hist_gray = 255 - (hist / np.amax(hist) * 255).astype(np.uint8)
    img_hist_rgb = grey_2_rgb(img_hist_gray)  # (255*mapping.to_rgba(img_hist_gray)[:, :, 0:3]).astype(np.uint8)

    # rescale based on scala param
    new_shape = (shape[1], shape[0])  # [width, height]
    img_hist_rgb = cv2.resize(img_hist_rgb, new_shape, interpolation=cv2.INTER_NEAREST)

    # overlay with histogram
    fig = plot_segments(img_hist_rgb, background, tomato, peduncle, show_background=True, alpha=0.1, linewidth=1,
                        show_axis=True, use_image_colours=False)

    # plot cluster centers
    centers_hue = np.rad2deg(centers['hue'])
    centers_hue[centers_hue < 0] += 360
    hue_coords = (centers_hue / 2) / hue_step  # [rad]
    a_coords = (centers['a'] - a_min) / a_step

    for label in lbl:
        i = lbl[label]
        if label == 'tomato':
            color = 'r'
        elif label == 'peduncle':
            color = 'g'
        elif label == 'background':
            color = 'b'
        plt.plot(hue_coords[i], a_coords[i], 'o', color=color, markeredgecolor='w', markersize=8, markeredgewidth=1,
                 alpha=1.0)

    # fix axis, only plot x axis for last image, only plot title for first
    if name == final_image_id:
        plt.xticks([hue_min, hue_height / 2, hue_height - 1], map(str, (0, 180, 360)))
        plt.xlabel("hue [$^\circ$]")
    else:
        plt.xticks([])

    plt.ylabel("a*")
    plt.yticks([0, a_height / 2, a_height - 1], map(str, (a_min, (a_min + a_max) / 2, a_max)))

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
