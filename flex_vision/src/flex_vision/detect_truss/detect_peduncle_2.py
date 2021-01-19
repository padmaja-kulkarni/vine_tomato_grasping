#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import os

from skimage.morphology import skeletonize
import skan
import scipy.sparse.csgraph as csgraph
from matplotlib import pyplot as plt
from matplotlib import animation

from sklearn import linear_model

from flex_vision.utils.util import add_circles, add_arrows, add_contour, add_strings
from flex_vision.utils.util import plot_image, save_fig
from flex_vision.utils.util import remove_blobs, bin2img, img2bin
from sklearn.metrics.pairwise import euclidean_distances
import matplotlib as mpl

NEW_PATH_COLOUR = [130, 50, 230]
JUNC_COLOR = (100, 0, 200)
END_COLOR = (200, 0, 0)

def detect_peduncle(peduncle_img, settings=None, px_per_mm=None, bg_img=None, save=False, name="", pwd=""):
    if settings is None:
        settings = settings.detect_peduncle()

    if (bg_img is not None) and save:
        fig = plt.figure()
        plot_image(bg_img)
        save_fig(fig, pwd, name + "_00")

    if bg_img is None:
        bg_img = peduncle_img

    if px_per_mm:
        branch_length_min_px = px_per_mm * settings['branch_length_min_mm']
    else:
        branch_length_min_px = settings['branch_length_min_px']

    skeleton_img = skeletonize_img(peduncle_img)
    junc_coords, end_coords = get_node_coord(skeleton_img)

    if save:
        visualize_skeleton(bg_img, skeleton_img, coord_junc=junc_coords, coord_end=end_coords,
                           name=name + "_01", pwd=pwd)

    if save:
        fit_ransac(peduncle_img, bg_img=bg_img.copy(), save=True, name=name + "_ransac", pwd=pwd)

    # update_image = True
    # while update_image:
    skeleton_img, b_remove = threshold_branch_length(skeleton_img, branch_length_min_px)
    # update_image = b_remove.any()
    junc_coords, end_coords = get_node_coord(skeleton_img)

    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, coord_junc=junc_coords, coord_end=end_coords,
                           name=name + "_02", pwd=pwd)

    graph, pixel_coordinates, degree_image = skan.skeleton_to_csgraph(skeleton_img, unique_junctions=True)
    dist, pred = csgraph.shortest_path(graph, directed=False, return_predecessors=True)

    end_nodes = coords_to_nodes(pixel_coordinates, end_coords[:, [1, 0]])
    junc_nodes = coords_to_nodes(pixel_coordinates, junc_coords[:, [1, 0]])

    path, path_length_px, branch_data = find_path(dist, pred, junc_nodes, end_nodes, pixel_coordinates, bg_image=bg_img.copy(), do_animate=False)

    branch_data = get_branch_center(branch_data, dist, pixel_coordinates, skeleton_img)

    path_img = path_mask(path, pixel_coordinates, skeleton_img.shape)
    junc_coords = pixel_coordinates[get_ids_on_path(path, pixel_coordinates, junc_nodes)][:, [1, 0]]
    end_coords = pixel_coordinates[get_ids_on_path(path, pixel_coordinates, end_nodes)][:, [1, 0]]

    end_coords = np.array([pixel_coordinates[path[0]][[1, 0]], pixel_coordinates[path[-1]][[1, 0]]])

    # make sure that end nodes are not labeled as junctions
    if junc_coords.shape[0] != 0:
        for end_coord in end_coords:
            dst = distance(junc_coords, end_coord)
            mask = dst > 0.1
            junc_coords = junc_coords[mask]

    if save:
        visualize_skeleton(bg_img, path_img, coord_junc=junc_coords,  # junc_nodes=junc_nodes, end_nodes=end_nodes,
                           coord_end=end_coords, name=name + "_03", pwd=pwd)

    if save:
        visualize_skeleton(bg_img, path_img, coord_junc=junc_coords, branch_data=branch_data, coord_end=end_coords,
                           name=name + "_04", pwd=pwd)

    return path_img, branch_data, junc_coords, end_coords


def find_path(dist, pred, junc_nodes, end_nodes, pixel_coordinates, bg_image=None, timeout=1000, do_animate=True):
    # initialize
    best_path = []
    best_length = 0

    if do_animate:
        start_nodes = [384]
    else:
        start_nodes = end_nodes

    for start_node in start_nodes:

        if do_animate:
            my_animation = PeduncleAnimation(pixel_coordinates, junc_nodes, end_nodes, bg_image=bg_image,
                                             name="animation_" + str(start_node))

        for end_node in end_nodes:

            count = 0
            angle_total = None
            from_node = start_node
            coord = pixel_coordinates[from_node]
            init_coord = coord
            length = 0

            path = []
            subpath = []  # TODO: was fist an empty list! from_node
            branch_data = []

            while count < timeout:

                from_node = pred[end_node, from_node]
                subpath.append(from_node)
                count += 1

                if from_node == -9999:
                    break

                if (from_node == junc_nodes).any() or np.any(from_node == end_nodes).any():
                    new_coord = pixel_coordinates[from_node]
                    angle_new = node_coord_angle(coord, new_coord)

                    if angle_total is None:
                        diff = 0
                    else:
                        diff = abs(angle_total - angle_new)

                    if diff < 45:
                        angle_total = node_coord_angle(init_coord, new_coord)  # angle_new

                        path.extend(subpath)
                        branch_data.append(subpath)
                        if len(subpath) == 0:
                            print subpath

                        subpath = []  # TODO: was fist an empty list! from_node

                    else:
                        # reset path
                        if len(path) > 1:
                            length = dist[(path[0], path[-1])]

                        if length >= best_length:
                            best_path = path
                            best_length = length
                            best_branch_data = branch_data

                        init_coord = coord
                        path = subpath
                        branch_data = []
                        branch_data.append(subpath)
                        subpath = []  # TODO: was fist an empty list! from_node
                        length = 0
                        angle_total = node_coord_angle(init_coord, new_coord)

                    coord = new_coord

                if do_animate:
                    my_animation.add_frame(path, subpath, start_node, end_node)

            if len(path) > 1:
                length = dist[(path[0], path[-1])]

            if length >= best_length:
                best_path = path
                best_length = length
                best_branch_data = branch_data

        if do_animate:
            my_animation.save()

    return best_path, best_length, best_branch_data


def get_locations_on_mask(mask, coords, allowable_distance=1):
    mask_coords = np.argwhere(mask)
    coord_keep = []

    for coord in coords:
        dist = np.sqrt(np.sum(np.power(mask_coords - coord[[1, 0]], 2), 1))
        if np.amin(dist) < allowable_distance:
            coord_keep.append(coord)

    return np.array(coord_keep)


def update_skeleton(skeleton_img, skeleton, i_remove):
    dtype = skeleton_img.dtype
    max_value = np.iinfo(dtype).max

    skeleton_prune_img = skeleton_img.copy()

    for i in i_remove:
        px_coords = skeleton.path_coordinates(i).astype(int)
        rows = px_coords[:, 0]
        cols = px_coords[:, 1]
        skeleton_prune_img[rows, cols] = 0

    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = cv2.dilate(skeleton_prune_img, kernel, iterations=1)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = remove_blobs(img)
    return bin2img(skeletonize(img / max_value))


def threshold_branch_length(skeleton_img, distance_threshold):
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)

    tip_junction = branch_data['branch-type'] == 1

    b_remove = (branch_data['branch-distance'] < distance_threshold) & tip_junction
    i_remove = b_remove.to_numpy().nonzero()[0]  # np.argwhere(b_remove)[:, 0]

    return update_skeleton(skeleton_img, skeleton, i_remove), b_remove


def get_node_coord(skeleton_img):
    """
    Finds all junction and end nodes on a provided skeleton
    """
    if np.all(skeleton_img == 0):
        return None, None

    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)

    # get all node IDs
    src_node_id = np.unique(branch_data['node-id-src'].values)
    dst_node_id = np.unique(branch_data['node-id-dst'].values)
    all_node_id = np.unique(np.append(src_node_id, dst_node_id))

    # get end and junc node IDs
    end_node_index = skeleton.degrees[all_node_id] == 1
    end_node_id = all_node_id[end_node_index]
    junc_node_id = np.setdiff1d(all_node_id, end_node_id)

    # get coordinates
    end_node_coord = skeleton.coordinates[end_node_id][:, [1, 0]]
    junc_node_coord = skeleton.coordinates[junc_node_id][:, [1, 0]]
    return junc_node_coord, end_node_coord


def distance(coords, coord):
    """ euclidean distance between coordinates two numpy array"""
    return np.sum((coords - coord) ** 2, axis=1)


def coords_to_nodes(pixel_coordinates, coords):
    nodes = []
    for coord in coords:
        nodes.append(np.argmin(distance(pixel_coordinates, coord)))

    return np.array(nodes)


def mean_absolute_deviation(data, axis=None):
    """
    mean absolute deviation
    """
    return np.mean(np.absolute(data - np.median(data, axis)), axis)


def get_path(from_node, to_node, pred, dist, timeout=1000):
    path = []
    length = 0
    count = 0
    while (to_node != -9999) and (count < timeout):
        count += 1
        path.append(to_node)
        length += dist[(from_node, to_node)]
        to_node = pred[(from_node, to_node)]

    if count > 0.9 * timeout:
        print count

    return path, length


def get_path_coordinates(path, pixel_coordinates):
    path_coordinates = np.rint(pixel_coordinates[path]).astype(int)
    col = path_coordinates[:, 1]
    row = path_coordinates[:, 0]
    return row, col

def node_coord_angle(src, dst):
    return np.rad2deg(np.arctan2((dst[0] - src[0]), (dst[1] - src[1])))


def skeletonize_img(img):
    return bin2img(skeletonize(img2bin(img)))


def get_branch_center(branch_data, dist_mat, pixel_coordinates, skeleton_img):
    loc = np.argwhere(skeleton_img)
    all_branch = {'junction-junction': [], 'junction-endpoint': []}
    n = len(branch_data)
    for i_branch, branch in enumerate(branch_data):

        src_id = branch[0]
        dst_id = branch[-1]

        src_node_coord = pixel_coordinates[dst_id]
        dst_node_coord = pixel_coordinates[src_id]

        angle = node_coord_angle(src_node_coord, dst_node_coord)
        length = dist_mat[src_id, dst_id]

        center = (dst_node_coord + src_node_coord) / 2
        dist = np.sqrt(np.sum(np.power(loc - center, 2), 1))
        i = np.argmin(dist)
        center_node_coord = loc[i]  # [loc[i, 0], loc[i, 1]]

        # branch_index = branch_data.index[i]
        coords = pixel_coordinates[branch]  ## skeleton.path_coordinates(i_row)
        new_branch = {'coords': coords[:, [1, 0]],
                      'src_node_coord': src_node_coord[[1, 0]],
                      'dst_node_coord': dst_node_coord[[1, 0]],
                      'center_node_coord': center_node_coord[[1, 0]],
                      'angle': angle,
                      'length': length}

        if i_branch == 0 or i_branch == (n - 1):  # junction-to-endpoint
            all_branch['junction-endpoint'].append(new_branch)
        else:
            all_branch['junction-junction'].append(new_branch)

    return all_branch


def path_mask(path, pixel_coordinates, shape):
    img = np.zeros(shape)

    if len(path) != 0:
        row, col = get_path_coordinates(path, pixel_coordinates)
        img[row, col] = 255

    return img.astype(np.uint8)


def visualize_skeleton(img, skeleton_img, skeletonize=False, coord_junc=None, coord_end=None, junc_nodes=None,
                       end_nodes=None, branch_data=None, name="", pwd=None, show_nodes=True, skeleton_color=None,
                       skeleton_width=4, show_img=True):

    if skeleton_color is None:
        skeleton_color = (0, 150, 30)

    if skeletonize:
        skeleton_img = skeletonize_img(skeleton_img)

    if show_img:
        fig = plt.figure()
        plot_image(img)
    else:
        fig = plt.gcf()

    add_contour(skeleton_img, skeleton_color, linewidth=skeleton_width, zorder=6)

    if (len(np.argwhere(skeleton_img)) > 2) and show_nodes:

        if (coord_junc is None) and (coord_end is None):
            coord_junc, coord_end = get_node_coord(skeleton_img)

        if coord_junc is not None:
            add_circles(coord_junc, radii=4, fc=JUNC_COLOR, linewidth=0, zorder=7)

        if coord_end is not None:
            add_circles(coord_end, radii=4, fc=END_COLOR, linewidth=0, zorder=7)

    if branch_data:
        branch_center = {}
        branch_angle = {}
        for branch_type in branch_data:
            branch_center[branch_type] = []
            branch_angle[branch_type] = []
            for branch in branch_data[branch_type]:
                branch_center[branch_type].append(branch['center_node_coord'])
                branch_angle[branch_type].append(branch['angle'])

        add_arrows(branch_center['junction-junction'], np.deg2rad(branch_angle['junction-junction']), color=JUNC_COLOR, linewidth=2)
        add_arrows(branch_center['junction-endpoint'], np.deg2rad(branch_angle['junction-endpoint']), color=END_COLOR, linewidth=2)

    if (junc_nodes is not None) or (end_nodes is not None):
        if junc_nodes is not None:
            for junc_node, coord in zip(junc_nodes, coord_junc):
                plt.text(coord[0], coord[1], str(junc_node))

        if end_nodes is not None:
            for end_node, coord in zip(end_nodes, coord_end):
                plt.text(coord[0], coord[1], str(end_node))

    if pwd:
        save_fig(fig, pwd, name)


def fit_ransac(peduncle_img, bg_img=None, save=False, name="", pwd=""):
    pend_color = np.array([0, 150, 30])
    stem_color = np.array([110, 255, 128])
    # skeletonize peduncle segment
    shape = peduncle_img.shape

    factor = 5
    mask_index = np.argwhere(peduncle_img)
    cols = mask_index[:, 1].reshape(-1, 1)  # x, spould be 2D
    rows = mask_index[:, 0]  # y

    i = range(0, len(rows), factor)
    fit_cols = cols[i]  # x, spould be 2D
    fit_rows = rows[i]  # y

    # Robustly fit linear model with RANSAC algorithm
    mad = mean_absolute_deviation(rows)
    residual_threshold = 0.8 * mad
    ransac = linear_model.RANSACRegressor(max_trials=1000, residual_threshold=residual_threshold)
    ransac.fit(fit_cols, fit_rows)

    # predict on full data set
    rows_pred = ransac.predict(cols)

    # idenitfy in and outliers
    data_train = np.stack((rows, cols[:, 0]), axis=1)
    data_pred = np.stack((rows_pred, cols[:, 0]), axis=1)
    dist = euclidean_distances(data_train, data_pred)
    inlier_mask = np.min(dist, axis=1) < residual_threshold
    outlier_mask = np.logical_not(inlier_mask)

    coord_inlier = [rows[inlier_mask], cols[inlier_mask, 0]]
    coord_outlier = [rows[outlier_mask], cols[outlier_mask, 0]]

    img_inlier = np.zeros(shape, dtype=np.uint8)
    img_outlier = np.zeros(shape, dtype=np.uint8)

    img_inlier[tuple(coord_inlier)] = 255
    img_outlier[tuple(coord_outlier)] = 255

    inlier_contours, hierarchy = cv2.findContours(img_inlier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    outlier_contours, hierarchy = cv2.findContours(img_outlier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    fig = plt.figure()
    plot_image(bg_img)

    for contour in inlier_contours:
        plt.plot(contour[:, 0, 0], contour[:, 0, 1], linestyle='-', linewidth=1, color=pend_color.astype(float) / 255)
    for contour in outlier_contours:
        plt.plot(contour[:, 0, 0], contour[:, 0, 1], linestyle='-', linewidth=1, color=stem_color.astype(float) / 255)

    line_cols = np.arange(fit_cols.min(), fit_cols.max())[:, np.newaxis]
    line_rows = ransac.predict(line_cols)
    plt.plot(line_cols, line_rows, color='navy', linewidth=2, label='RANSAC regressor')
    plt.legend(loc='lower right')
    if pwd is not None:
        save_fig(fig, pwd, name, title="RANSAC")

    return img_inlier


def get_ids_on_path(path, pixel_coordinates, node_ids, threshold=0.001):
    row, col = get_path_coordinates(path, pixel_coordinates)
    path_coordinates = np.column_stack([row, col])  # [row, col]
    node_coordinates = pixel_coordinates[node_ids]  # [row, col]

    dist = euclidean_distances(path_coordinates, node_coordinates)

    # return the node coordaine, thus index = 1
    i_keep = np.argwhere(dist < threshold)[:, 1]
    return node_ids[i_keep]


class PeduncleAnimation(object):

    def __init__(self, pixel_coordinates, junc_node_ids, tail_node_ids, bg_image=None, shape=None, name="animation",
                 save_path=None):
        self.artists = []
        self.fig, self.ax = plt.subplots()
        self.name = name
        self.pixel_coordinates = pixel_coordinates[:, [1, 0]]
        self.junc_node_ids = junc_node_ids
        self.tail_node_ids = tail_node_ids
        self.frame_limit = 1000

        self.n_calls = 0
        self.calls_per_frame = 5  # amount of calls per frame

        if save_path is None:
            self.save_path = os.path.dirname(os.path.realpath(__file__))
        else:
            self.save_path = save_path

        if bg_image is not None:
            self.bg_img = bg_image
            self.shape = bg_image.shape
        elif shape is not None:
            self.bg_img = np.zeros(shape)
            self.shape = shape
        else:
            print "Please provide either a shape or an background image"

        self.initialize_static_background()

    def initialize_static_background(self):
        """initializes the static animation background by plotting the background image, nodes and possible paths"""
        plot_image(self.bg_img, animated=True)

        junc_coords = self.pixel_coordinates[self.junc_node_ids]
        add_circles(junc_coords, radii=3, fc=JUNC_COLOR, linewidth=0, zorder=1, alpha=0.25)

        end_coords = self.pixel_coordinates[self.tail_node_ids]
        add_circles(end_coords, radii=3, fc=END_COLOR, linewidth=0, zorder=1, alpha=0.25)

        color = np.array(JUNC_COLOR).astype(float) / 255
        self.ax.scatter(self.pixel_coordinates[:, 0], self.pixel_coordinates[:, 1], color=color, linewidth=0, alpha=0.05)

    def add_frame(self, path, subpath, src_node_id, dst_node_id):
        """Add a frame"""

        # we only save one per N frames
        self.n_calls = self.n_calls % self.calls_per_frame
        if not self.n_calls == 0:
            self.n_calls += 1
            print "."
            return

        self.n_calls += 1

        n_frame = len(self.artists) + 1
        if n_frame > self.frame_limit:
            print "frame limit succeeded, refusing to add frame"
            return
        else:
            print "adding frame number: ", n_frame
        frame_artists = []

        color = np.array(JUNC_COLOR).astype(float) / 255
        if len(path) != 0:
            row, col = get_path_coordinates(path, self.pixel_coordinates[:, [1, 0]])
            path_plt, = self.ax.plot(col, row, color=color)
            frame_artists.append(path_plt)

        color = np.array(NEW_PATH_COLOUR).astype(float) / 255
        if subpath is not None:
            row, col = get_path_coordinates(subpath, self.pixel_coordinates[:, [1, 0]])
            path_plt, = self.ax.plot(col, row, color=color)
            frame_artists.append(path_plt)

        color = np.array(END_COLOR).astype(float) / 255
        for node_id in [src_node_id, dst_node_id]:
            node_coord = self.pixel_coordinates[node_id]
            tail_circle = mpl.patches.Circle(node_coord, 3, color=color, animated=True, zorder=1)
            tail_text = self.ax.text(node_coord[0] + 5, node_coord[1] - 5, node_id)
            frame_artists.append(self.ax.add_artist(tail_circle))
            frame_artists.append(tail_text)

        total_path = list(path)
        total_path.extend(subpath)
        junc_node_ids = get_ids_on_path(total_path, self.pixel_coordinates[:, [1, 0]], self.junc_node_ids)
        junc_node_coords = self.pixel_coordinates[junc_node_ids]
        color = np.array(JUNC_COLOR).astype(float) / 255
        for junc_coord, junc_id in zip(junc_node_coords, junc_node_ids):
            junc_circle = mpl.patches.Circle(junc_coord, 3, color=color, animated=True, zorder=1)
            junc_text = self.ax.text(junc_coord[0] + 5, junc_coord[1] - 5, junc_id)
            frame_artists.append(self.ax.add_artist(junc_circle))
            frame_artists.append(junc_text)

        self.artists.append(frame_artists)

    def save(self, extension='.mp4', dpi=200):
        """Save and close figure"""
        file_name = os.path.join(self.save_path, self.name) + extension
        print "Saving animation to file" + file_name + "..."
        ani = animation.ArtistAnimation(self.fig, self.artists)  # , interval=50, blit=True, repeat_delay=1000
        writer = animation.FFMpegWriter(fps=30, metadata=dict(artist='Taeke de Haan'), bitrate=-1)
        ani.save(file_name, writer, dpi=dpi)
        plt.close()
        print "Done saving!"