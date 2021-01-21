# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 21:21:28 2020

@author: taeke
"""

import os
import cv2
import numpy as np
import warnings
import copy
from matplotlib import pyplot as plt
import matplotlib as mpl

from flex_vision.utils import color_maps

from matplotlib.backends.backend_agg import FigureCanvasAgg

ee_color = (255, 150, 0)
grasp_color = (200, 0, 150)
tomato_color = (255, 82, 82)
peduncle_color = (82, 255, 82)  # (0, 150, 30)
background_color = (0, 0, 255)
junction_color = (100, 0, 200)
end_color = (200, 0, 0)
gray_color = (150, 150, 150)

background_layer = 0
bottom_layer = 1  # contours
middle_layer = 5  # tomatoes
peduncle_layer = 6
vertex_layer = 7
high_layer = 8  # arrows, com
top_layer = 10  # junctions, com, text

default_ext = 'png'


def make_dirs(pwd):
    if not os.path.isdir(pwd):
        print("New path, creating a new folder: " + pwd)
        os.makedirs(pwd)


def load_rgb(name, pwd=None, horizontal=True):
    """ load image """
    if pwd is None:
        name_full = os.path.join(name)
    else:
        name_full = os.path.join(pwd, name)

    if not os.path.isfile(name_full):
        print('Cannot load RGB data from file: ' + name_full + ', as it does not exist!')
        return None

    img_bgr = cv2.imread(name_full)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    shape = img_rgb.shape[:2]

    # transpose image if required
    if horizontal and (shape[0] > shape[1]):
        img_rgb = np.transpose(img_rgb, [1, 0, 2])

    if img_rgb is None:
        print("Failed to load image from path: %s" % name_full)

    return img_rgb


def bin2img(binary, dtype=np.uint8, copy=False):
    """
    convert a binary image to the desired data type, default: uint8
    """
    max_value = np.iinfo(dtype).max
    return binary.astype(dtype, copy=copy) * max_value


def img2bin(img, copy_img=False):
    """
    convert image to binary with boolean data type
    """
    return img.astype(bool, copy=copy_img)


def change_brightness(img, brightness):
    img_copy = img.copy()

    if 0 < brightness < 1:
        return img_copy + ((255 - img_copy) ** brightness).astype(np.uint8)

    if -1 < brightness < 0:
        return img_copy - (img_copy ** -brightness).astype(np.uint8)

    else:
        print 'I can not do anything with a brightness value of ', brightness, '!'
        return img

def change_color_brightness(color, brightness):
    color = np.array(color)

    if 0 < brightness < 1:
        return color + ((255 - color) ** brightness).astype(np.uint8)

    if -1 < brightness < 0:
        return color - (color ** -brightness).astype(np.uint8)

    else:
        print 'I can not do anything with a brightness value of ', brightness, '!'
        return color


def angular_difference(x, y):
    """
    compute the difference between two angles x, y
    """
    return np.abs(np.arctan2(np.sin(x - y), np.cos(x - y)))


def remove_blobs(img_in):
    dtype = img_in.dtype
    value = np.iinfo(dtype).max

    # initialize outgoing image
    img_out = np.zeros(img_in.shape[:2], dtype)

    # the extra [-2:] is essential to make it work with varios cv2 ersions
    contours, _ = cv2.findContours(img_in, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]

    # only when the image contains a contour
    if len(contours) != 0:
        # print('Filling largest blob...')
        cnt = max(contours, key=cv2.contourArea)
        cv2.drawContours(img_out, [cnt], -1, value, cv2.FILLED)
        # print('Done!...')
    return cv2.bitwise_and(img_in, img_out)


def add_border(imOriginal, location, sizeBorder):
    sizeOriginal = imOriginal.shape
    location = location.astype(int)

    imBorder = np.zeros(sizeBorder[0:2], np.uint8)

    colStart = location[0, 0]
    colEnd = location[0, 0] + sizeOriginal[1]

    rowStart = location[0, 1]
    rowEnd = location[0, 1] + sizeOriginal[0]

    if (rowEnd > sizeBorder[0]):
        warnings.warn('cutting immage')
        rowEnd = sizeBorder[0]

    imBorder[rowStart:rowEnd, colStart:colEnd] = imOriginal[0:rowEnd - rowStart, :]

    return imBorder


def label_img(data, centers):
    dist = abs(data - np.transpose(centers))
    labels = np.argmin(dist, 1).astype(np.int32)
    return np.expand_dims(labels, axis=1)


def stack_segments(imRGB, background, tomato, peduncle, use_image_colours=True):
    # stack segments

    [h, w] = imRGB.shape[:2]

    # set labels
    backgroundLabel = 0
    tomatoLabel = 1
    peduncleLabel = 2

    # label pixels
    pixelLabel = np.zeros((h, w), dtype=np.int8)
    pixelLabel[background > 0] = backgroundLabel
    pixelLabel[cv2.bitwise_and(tomato, cv2.bitwise_not(peduncle)) > 0] = tomatoLabel
    pixelLabel[peduncle > 0] = peduncleLabel

    # get class colors
    if use_image_colours:
        color_background = np.uint8(np.mean(imRGB[pixelLabel == backgroundLabel], 0))
        color_tomato = np.uint8(np.mean(imRGB[pixelLabel == tomatoLabel], 0))
        color_peduncle = np.uint8(np.mean(imRGB[pixelLabel == peduncleLabel], 0))

    else:
        color_background = background_color
        color_tomato = tomato_color
        color_peduncle = peduncle_color

    color = np.vstack([color_background, color_tomato, color_peduncle]).astype(np.uint8)
    # visualize
    res = color[pixelLabel.flatten()]
    res2 = res.reshape((h, w, 3))

    return res2


def grey_2_rgb(img_grey, vmin=0, vmax=255):
    norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
    cmap = mpl.cm.hot
    mapping = mpl.cm.ScalarMappable(norm=norm, cmap=cmap)
    img_rgb = (vmax * mapping.to_rgba(img_grey)[:, :, 0:3]).astype(np.uint8)
    return img_rgb


def save_img(img, pwd, name, resolution=300, title="", title_size=20, ext=None, color_map='plasma', vmin=None,
             vmax=None):
    if ext is None:
        ext = default_ext

    plt.rcParams["savefig.format"] = ext
    plt.rcParams["savefig.bbox"] = 'tight'
    plt.rcParams['axes.titlesize'] = title_size
    # plt.rcParams['image.cmap'] = color_map

    if color_map == 'HSV':
        color_map = color_maps.hsv_color_scale()
    elif color_map == 'Lab':
        color_map = color_maps.lab_color_scale()

    fig = plt.figure()
    plt.imshow(img, cmap=color_map, vmin=vmin, vmax=vmax)
    plt.axis('off')
    if title is not None:
        plt.title(title)

    # https://stackoverflow.com/a/27227718
    plt.gca().set_axis_off()
    plt.subplots_adjust(top=1, bottom=0, right=1, left=0,
                        hspace=0, wspace=0)
    plt.margins(0, 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())

    # make dir if it does not yet exist
    make_dirs(pwd)
    fig.savefig(os.path.join(pwd, name), dpi=resolution, bbox_inches='tight', pad_inches=0)
    plt.close(fig)


def save_fig(fig, pwd, name, resolution=300, no_ticks=True, title="", titleSize=20, ext=None):
    if ext is None:
        ext = default_ext

    # eps does not support transparancy
    plt.rcParams["savefig.format"] = ext
    plt.rcParams["savefig.bbox"] = 'tight'

    for ax in fig.get_axes():
        pass
        # ax.label_outer()

    if no_ticks:
        for ax in fig.get_axes():
            # ax.yaxis.set_major_locator(plt.nulllocator())\
            ax.set_yticklabels([])
            ax.set_xticklabels([])
    # else:
        # We change the fontsize of minor ticks label
        # ax.tick_params(axis='both', which='major', labelsize=10)
        # ax.tick_params(axis='both', which='minor', labelsize=8)

    plt.margins(0, 0)

    # make dir if it does not yet exist
    make_dirs(pwd)
    fig.savefig(os.path.join(pwd, name), dpi=resolution, bbox_inches='tight', pad_inches=0)
    plt.close(fig)


def plot_segments(img_rgb, background, tomato, peduncle, fig=None, show_background=False, pwd=None,
                  use_image_colours=True, show_axis=False, name=None, title="", alpha=0.4, linewidth=0.5):
    """
        alpha: trasparancy of segments, low value is more transparant!
    """

    if fig is None:
        fig = plt.figure()

    img_segments = stack_segments(img_rgb, background, tomato, peduncle, use_image_colours=use_image_colours)
    added_image = cv2.addWeighted(img_rgb, 1 - alpha, img_segments, alpha, 0)
    plot_image(added_image, show_axis=show_axis)

    # plot all contours
    if show_background:
        add_contour(background, color=background_color, linewidth=linewidth)
    add_contour(tomato, color=tomato_color, linewidth=linewidth)
    add_contour(peduncle, color=peduncle_color, linewidth=linewidth)

    if pwd is not None:
        save_fig(fig, pwd, name, title=title)

    return fig


def plot_image(img, show_axis=False, animated=False):
    """
        plot image
    """
    plt.imshow(img, animated=animated)
    if not show_axis:
        clear_axis()

def clear_axis():
    plt.tight_layout()
    plt.axis('off')

    # https://stackoverflow.com/a/27227718
    plt.gca().set_axis_off()
    plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
    plt.margins(0, 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())


def plot_truss(img_rgb=None, tomato=None, peduncle=None):
    if img_rgb is not None:
        plt.figure()
        plot_image(img_rgb)

    if tomato:
        add_circles(tomato['centers'], radii=tomato['radii'], fc=tomato_color, ec=tomato_color)

    if peduncle:
        add_lines(peduncle['centers'], peduncle['angles'], lengths=peduncle['length'], color=peduncle_color, linewidth=20)


def plot_features(img_rgb=None, tomato=None, peduncle=None, grasp=None,
                  alpha=0.4, linewidth=1, zoom=False, pwd=None, file_name=None, title=""):
    if img_rgb is not None:
        fig = plt.figure()
        plot_image(img_rgb)
    else:
        fig = plt.gcf()

    if zoom:
        tom_linestyle = (0, (10, 10))
        tom_width = 2
        com_radius = 16
        junc_radius = 8
    else:
        tom_linestyle = (0, (5, 5))
        tom_width = 2
        com_radius = 8
        junc_radius = 8

    if tomato:
        add_circles(tomato['centers'], radii=tomato['radii'], fc=tomato_color, linewidth=tom_width, alpha=alpha,
                    linestyle=tom_linestyle)

        if 'com' in tomato.keys():
            add_com(tomato['com'], radius=com_radius)

    if peduncle:
        add_circles(peduncle['junctions'], radii=junc_radius, fc=junction_color, linewidth=linewidth, zorder=top_layer)

    if grasp:
        col = grasp['col']
        row = grasp['row']
        angle = grasp['angle']
        plot_grasp_location([[col, row]], angle, finger_width=20, finger_thickness=15, linewidth=2)

    if pwd is not None:
        save_fig(fig, pwd, file_name, title=title)


def plot_features_result(img_rgb, tomato_pred=None, peduncle=None, grasp=None,
                         alpha=0.5, linewidth=1, zoom=False, pwd=None, name=None, title=""):
    fig = plt.figure()
    plot_image(img_rgb)

    if zoom:
        tom_linestyle = (0, (10, 10))
        com_radius = 10
        junc_radius = 8
    else:
        tom_linestyle = (0, (5, 5))
        com_radius = 10
        junc_radius = 8

    if tomato_pred:
        add_circles(tomato_pred['true_pos']['centers'], radii=tomato_pred['true_pos']['radii'], fc=tomato_color,
                    linewidth=2, alpha=alpha, linestyle=tom_linestyle)
        add_circles(tomato_pred['false_pos']['centers'], radii=tomato_pred['false_pos']['radii'], fc=tomato_color,
                    linewidth=linewidth, alpha=alpha, linestyle=tom_linestyle)
        add_com(tomato_pred['com'], radius=com_radius)

    if peduncle:
        add_circles(peduncle['false_pos']['centers'], radii=junc_radius, ec=(255, 0, 0), linewidth=linewidth, alpha=0,
                    zorder=top_layer)
        add_circles(peduncle['true_pos']['centers'], radii=junc_radius, fc=junction_color, linewidth=linewidth,
                    zorder=top_layer)

    if grasp:
        col = grasp['col']
        row = grasp['row']
        angle = grasp['angle']
        plot_grasp_location([[col, row]], angle, finger_width=20, finger_thickness=15, linewidth=2)

    if pwd is not None:
        save_fig(fig, pwd, name, title=title)


def plot_timer(timer_dict, N=1, threshold=0, ignore_key=None, pwd=None, name='time', title='time', startangle=-45):
    for key in timer_dict.keys():

        # remove ignored keys
        if key == ignore_key:
            del timer_dict[key]
            continue

        # remove empty keys
        if timer_dict[key] == []:
            del timer_dict[key]

    values = np.array(timer_dict.values())

    if len(values.shape) == 1:
        values = values / N
    else:
        values = np.mean(values, axis=1)

    labels = np.array(timer_dict.keys())

    values_rel = values / np.sum(values)
    i_keep = (values_rel > threshold)
    i_remove = np.bitwise_not(i_keep)

    # if everything is put under others
    if np.all(i_remove is True):
        print("No time to plot!")
        return

    labels_keep = labels[i_keep].tolist()
    values_keep = values[i_keep].tolist()

    if np.any(i_remove is True):
        remains = np.mean(values[i_remove])
        values_keep.append(remains)
        labels_keep.append('others')

    l = zip(values_keep, labels_keep)
    l.sort()
    values_keep, labels_keep = zip(*l)

    donut(values_keep, labels_keep, pwd=pwd, name=name, title=title, startangle=startangle)


def donut(data, labels, pwd=None, name=None, title=None, startangle=-45):
    data = np.array(data)
    data_rel = data / sum(data) * 100

    text = []
    separator = ': '
    for label, value, value_rel in zip(labels, data, data_rel):
        text.append(label + separator + str(int(round(value_rel))) + '% (' + str(int(round(value))) + ' ms)')

    fig, ax = plt.subplots(figsize=(6, 3), subplot_kw=dict(aspect="equal"))

    wedges, texts = ax.pie(data, wedgeprops=dict(width=0.5), startangle=startangle)

    bbox_props = dict(boxstyle="round,pad=0.3", fc=[0.92, 0.92, 0.92], lw=0)  # square, round
    kw = dict(arrowprops=dict(arrowstyle="-"),
              bbox=bbox_props, zorder=0, va="center", fontsize=15)

    y_scale = 1.8

    for i, p in enumerate(wedges):
        ang = (p.theta2 - p.theta1) / 2. + p.theta1
        y = np.sin(np.deg2rad(ang))
        x = np.cos(np.deg2rad(ang))
        horizontalalignment = {-1: "right", 1: "left"}[int(np.sign(x))]
        connectionstyle = "angle,angleA=0,angleB={}".format(ang)
        kw["arrowprops"].update({"connectionstyle": connectionstyle})
        ax.annotate(text[i], xy=(x, y), xytext=(1.35 * np.sign(x), y_scale * y),
                    horizontalalignment=horizontalalignment, **kw)

    ax.set_title(title, fontsize=20)
    plt.tight_layout()

    if pwd is not None:
        save_fig(fig, pwd, name, no_ticks=False)


def plot_grasp_location(loc, angle, finger_width=20, finger_thickness=10, finger_dist=None, linewidth=1, pwd=None, name=None, title=''):
    """
        angle in rad
    """

    if isinstance(loc, (list, tuple, np.matrix)):
        loc = np.array(loc, ndmin=2)

    if angle is None:
        return

    if len(loc.shape) > 1:
        if loc.shape[0] > loc.shape[1]:
            loc = loc.T

        loc = loc[0]

    if (loc[0] is None) or (loc[1] is None):
        return

    rot_angle = angle + np.pi / 2
    if finger_dist is not None:
        right_origin, left_origin = compute_line_points(loc, rot_angle, finger_dist)

    else:
        left_origin = loc
        right_origin = loc


    R = np.array([[np.cos(rot_angle), -np.sin(rot_angle)], [np.sin(rot_angle), np.cos(rot_angle)]])

    xy = np.array([[-finger_thickness], [-finger_width/2]])
    xy_rot = np.matmul(R, xy) + np.expand_dims(left_origin, axis = 1)

    add_rectangle(xy_rot, finger_thickness, finger_width, angle=np.rad2deg(rot_angle), ec=ee_color,
                  fc=ee_color, alpha=0.4, linewidth=linewidth, zorder=middle_layer)

    if finger_dist is not None:
        add_rectangle(xy_rot, 2*finger_thickness + finger_dist, finger_width, angle=np.rad2deg(rot_angle), ec=grasp_color,
                  fc=grasp_color, alpha=0.4, linewidth=linewidth, linestyle='-', zorder=bottom_layer)

    xy = np.array([[0], [-finger_width/2]])
    xy_rot = np.matmul(R, xy) + np.expand_dims(right_origin, axis=1)

    add_rectangle(xy_rot, finger_thickness, finger_width, angle=np.rad2deg(rot_angle), ec=ee_color,
                  fc=ee_color, alpha=0.4, linewidth=linewidth, zorder=middle_layer)


    if pwd is not None:
        save_fig(plt.gcf(), pwd, name, title=title)


def plot_error(tomato_pred, tomato_act, error,
               pwd=None,
               name=None,
               use_mm=False,
               title="",
               resolution=300,
               title_size=20,
               ext=None):
    if ext is None:
        ext = default_ext

    fig = plt.gcf()
    ax = plt.gca()

    plt.rcParams["savefig.format"] = ext
    plt.rcParams["savefig.bbox"] = 'tight'
    plt.rcParams['axes.titlesize'] = title_size

    plt.title(title)

    if use_mm:
        unit = 'mm'
    else:
        unit = 'px'

    n_true_pos = len(tomato_pred['true_pos']['centers'])
    n_false_pos = len(tomato_pred['false_pos']['centers'])
    n_false_neg = len(tomato_act['false_neg']['centers'])
    if 'com' in tomato_pred.keys():
        n_com = len(tomato_pred['com'])
    else:
        n_com = 0

    centers = []
    centers.extend(tomato_pred['true_pos']['centers'])
    centers.extend(tomato_pred['false_pos']['centers'])
    centers.extend(tomato_act['false_neg']['centers'])
    if 'com' in tomato_pred.keys():
        centers.extend([tomato_pred['com']])

    labels = []
    labels.extend(['true_pos'] * n_true_pos)
    labels.extend(['false_pos'] * n_false_pos)
    labels.extend(['false_neg'] * n_false_neg)
    labels.extend(['com'] * n_com)

    error_centers = []
    error_centers.extend(error['centers'])
    error_centers.extend(n_false_pos * [None])
    error_centers.extend(n_false_neg * [None])
    if 'com' in tomato_pred.keys():
        error_centers.append(error['com'])

    if 'radii' in error.keys():
        error_radii_val = error['radii']
    else:
        error_radii_val = [None] * n_true_pos
    error_radii = []
    error_radii.extend(error_radii_val)
    error_radii.extend(n_false_pos * [None])
    error_radii.extend(n_false_neg * [None])
    error_radii.extend(n_com * [None])

    # sort based on the y location of the centers
    zipped = zip(centers, error_centers, error_radii, labels)
    zipped.sort(key=lambda x: x[0][1])
    centers, error_centers, error_radii, labels = zip(*zipped)

    # default bbox style
    bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="w", lw=0.72)
    kw_default = dict(arrowprops=dict(arrowstyle="-"),
                      bbox=bbox_props, va="center", size=12, color='k')

    y_lim = ax.get_ylim()
    h = y_lim[0] - y_lim[1]
    x_lim = ax.get_xlim()
    w = x_lim[1] - x_lim[0]

    # h, w = img.shape[:2]
    n = len(centers) + 1
    y_text = 0  # 1.0/n* h
    for center, error_center, error_radius, label in zip(centers, error_centers, error_radii, labels):

        # copy default style
        kw = copy.deepcopy(kw_default)

        if label == 'true_pos':

            center_error = int(round(error_center))
            if error_radius is not None:
                radius_error = int(round(error_radius))
                text = 'loc: {c:d}{u:s} \nr: {r:d}{u:s}'.format(c=center_error, r=radius_error, u=unit)
            else:
                text = 'loc: {c:d}{u:s}'.format(c=center_error, u=unit)
            arrow_color = 'k'

        elif label == 'com':
            center_error = int(round(error_center))
            text = 'com: {c:d}{u:s}'.format(c=center_error, u=unit)
            kw['bbox']['fc'] = 'k'
            kw['bbox']['ec'] = 'k'
            kw['color'] = 'w'
            arrow_color = 'k'

        elif label == 'false_pos':
            text = 'false positive'
            kw['bbox']['fc'] = 'r'
            kw['bbox']['ec'] = 'r'
            arrow_color = 'r'

        elif label == 'false_neg':
            text = 'false negative'
            kw['bbox']['ec'] = 'lightgrey'
            kw['bbox']['fc'] = 'lightgrey'
            arrow_color = 'lightgrey'

        # print center, label
        y = center[1]
        x = center[0]

        if x <= 0.35 * w:
            x_text = 0.6 * w  # -0.2*w
        elif x <= 0.5 * w:
            x_text = 0.2 * w * 0.25
        elif x <= 0.65 * w:
            x_text = 0.8 * w
        else:
            x_text = 0.2 * w  # w

        y_text = y_text + 1.0 / n * h

        x_diff = x_text - x
        y_diff = y_text - y
        if (x_diff > 0 and y_diff > 0) or (x_diff < 0 and y_diff < 0):
            ang = -45
        else:
            ang = 45

        connectionstyle = "angle,angleA=0,angleB={}".format(ang)
        kw["arrowprops"].update({"connectionstyle": connectionstyle, 'color': arrow_color})
        plt.annotate(text, xy=(x, y), xytext=(x_text, y_text), zorder=high_layer, **kw)  #

    if pwd:
        fig.savefig(os.path.join(pwd, name), dpi=resolution, bbox_inches='tight', pad_inches=0)


def compute_line_points(center, angle, l):
    """
        angle in rad
    """
    col = center[0]
    row = center[1]

    col_start = col + 0.5 * l * np.cos(angle)
    row_start = row + 0.5 * l * np.sin(angle)

    col_end = col - 0.5 * l * np.cos(angle)
    row_end = row - 0.5 * l * np.sin(angle)

    start_point = np.array([col_start, row_start])
    end_point = np.array([col_end, row_end])
    return start_point, end_point


def add_com(center, radius=5):
    """
        center: circle centers expressed in [col, row]
    """
    ax = plt.gca()
    center = np.array(center, ndmin=2)

    pwd = os.path.dirname(__file__)
    pwd_img = os.path.join(pwd, '..', 'images')
    img = mpl.image.imread(os.path.join(pwd_img, 'com.png'))

    com_radius, _, _ = img.shape
    com_zoom = radius / float(com_radius)
    imagebox = mpl.offsetbox.OffsetImage(img, zoom=com_zoom)

    props = dict(alpha=0, zorder=top_layer)
    ab = mpl.offsetbox.AnnotationBbox(imagebox, (center[0, 0], center[0, 1]), pad=0, bboxprops=props)
    ab.set_zorder(high_layer)
    ax.add_artist(ab)


def add_strings(strings, locations):
    """
    Add strings to current axis
    locations -- [col, row]
    """
    for string, location in zip(strings, locations):
        plt.text(location[0], location[1], string)


def add_circles(centers, radii=5, fc=(255, 255, 255), ec=(0, 0, 0), linewidth=1, alpha=1.0, linestyle='-', zorder=None,
                pwd=None, name=None, title="", get_artist=False):
    """
        centers: circle centers expressed in [col, row]
    """

    if zorder is None:
        zorder = middle_layer

    if isinstance(centers, (list, tuple, np.matrix)):
        centers = np.array(centers, ndmin=2)

    # if a single radius is give, we repeat the value
    if not isinstance(radii, (list, np.ndarray)):
        radii = [radii] * centers.shape[0]

    if len(centers.shape) == 1:
        centers = np.array(centers, ndmin=2)

    # if empty we can not add any circles
    if centers.shape[0] == 0:
        return

    if centers.shape[1] == 0:
        return

    fc = np.array(fc).astype(float) / 255
    ec = np.array(ec).astype(float) / 255

    fc = np.append(fc, alpha)
    ec = np.append(ec, 1)

    # centers should be integers
    centers = np.round(centers).astype(dtype=int)  # (col, row)
    radii = np.round(radii).astype(dtype=int)  # (col, row)

    for center, radius in zip(centers, radii):
        circle = mpl.patches.Circle(center, radius, ec=ec, fc=fc, fill=True, linewidth=linewidth,
                                           linestyle=linestyle, zorder=zorder)

        ax = plt.gca()
        artist = ax.add_artist(circle)

    if pwd is not None:
        save_fig(plt.gcf(), pwd, name, title="", titleSize=20)

    return artist

def add_contour(mask, color=(255, 255, 255), linewidth=1, zorder=None):
    if zorder is None:
        zorder = bottom_layer

    color = np.array(color).astype(float) / 255
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    for contour in contours:
        plt.plot(contour[:, 0, 0], contour[:, 0, 1], linestyle='-', linewidth=linewidth, color=color,
                 zorder=zorder)


def add_lines(centers, angles, lengths=20, color=(255, 255, 255), linewidth=1, is_rad=True):
    """
    angle in rad
    """
    if isinstance(centers, (list, tuple)):
        centers = np.array(centers, ndmin=2)

    if len(centers.shape) == 1:
        centers = np.array(centers, ndmin=2)

    if not isinstance(angles, (list, tuple)):
        angles = [angles]

    if not isinstance(color, (str)):
        color = np.array(color).astype(float) / 255

    if not isinstance(lengths, (list, tuple)):
        lengths = [lengths] * len(angles)

    for center, angle, length in zip(centers, angles, lengths):
        if not is_rad:
            angle = angle / 180 * np.pi

        start_point, end_point = compute_line_points(center, angle, length)
        plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], color=color, linewidth=linewidth,
                 zorder=top_layer)


def add_lines_from_points(start_point, end_point, color=(255, 255, 255), linewidth=1, linestyle='-'):
    if not isinstance(color, (str)):
        color = np.array(color).astype(float) / 255

    plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], color=color, linewidth=linewidth,
             linestyle=linestyle, zorder=top_layer)


def add_arrows(centers, angles, lengths=20, color=(255, 255, 255), linewidth=1, head_width=5, head_length=7, is_rad=True):
    """
    angle in rad
    """
    if isinstance(centers, (list, tuple)):
        centers = np.array(centers, ndmin=2)

    if not isinstance(angles, (list, tuple, np.ndarray)):
        angles = [angles]

    if not isinstance(lengths, (list, tuple, np.ndarray)):
        lengths = [lengths]

    color = np.array(color).astype(float) / 255

    for center, angle, length in zip(centers, angles, lengths):
        if not is_rad:
            angle = angle / 180 * np.pi
        start_point, end_point = compute_line_points(center, angle, length)

        plt.arrow(start_point[0], start_point[1], end_point[0] - start_point[0], end_point[1] - start_point[1],
                  color=color, lw=linewidth, head_width=head_width, head_length=head_length, zorder=top_layer)


def add_rectangle(xy, width, height, angle=0, fc=(255, 255, 255), ec=(0, 0, 0), linewidth=1, alpha=1.0, linestyle='-', zorder=None,
                pwd=None, name=None, title=""):

    if zorder is None:
        zorder = middle_layer

    fc = np.array(fc).astype(float) / 255
    ec = np.array(ec).astype(float) / 255

    fc = np.append(fc, alpha)
    ec = np.append(ec, 1)

    rectangle = mpl.patches.Rectangle(xy, width, height, angle=angle, ec=ec, fc=fc, fill=True, linewidth=linewidth,
                                      linestyle=linestyle, zorder=zorder)
    ax = plt.gca()
    ax.add_artist(rectangle)


def figure_to_image(fig):
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    s, (width, height) = canvas.print_to_buffer()

    # Option 2a: Convert to a NumPy array.
    img = np.fromstring(s, np.uint8).reshape((height, width, 4))
    return img
