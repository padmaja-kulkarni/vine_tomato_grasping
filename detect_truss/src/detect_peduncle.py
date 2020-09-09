# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:16:42 2020

@author: taeke
"""

import os
from detect_truss.util import make_dirs, load_rgb
from detect_truss.util import change_brightness, plot_timer

from detect_truss.ProcessImage import ProcessImage
from detect_truss.detect_peduncle import visualize_skeleton, detect_peduncle, threshold_branch_length, skeletonize_img

from detect_truss.timer import Timer
from detect_truss.counter import Counter

import numpy as np
from skimage.morphology import skeletonize
import skan
import scipy.sparse.csgraph as csgraph
import scipy.optimize as optimize
from matplotlib import pyplot as plt

from scipy.interpolate import UnivariateSpline


def func(x, a, b, c):
    return a*x**2 + b*x + c
    # return (r**2 - (x - x0)**2)**0.5 + y0


def coords_to_nodes(pixel_coordinates, coords):
    nodes = []
    for coord in coords:
        nodes.append(np.argmin(np.sum((pixel_coordinates - coord) ** 2, axis=1)))

    return np.array(nodes)

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
    path_coordinates = np.rint(pixel_coordinates[path]).astype(int)  #
    col = path_coordinates[:, 1]
    row = path_coordinates[:, 0]

    return row, col

def get_fit_error(row_true, col_true, param):

    row_fit = func(col_true, *param)
    return np.sum((row_true - row_fit) ** 2)**0.5

def show_path(path, pixel_coordinates, shape, param=None, show=False):
    img = np.zeros(shape)
    row, col = get_path_coordinates(path, pixel_coordinates)
    img[row, col] = 255

    if param is not None:
        # col = np.array(range(0, shape[1]))
        row = func(col, *param)

        # col = np.rint(col).astype(int)
        row = np.rint(row).astype(int)

        i_keep = np.logical_and(row > 0, row < shape[0])

        img[row[i_keep], col[i_keep]] = 125

    if show:
        plt.figure()
        plt.imshow(img)
        plt.show()

    return img.astype(np.uint8)


def node_coord_angle(src, dst):
    return np.rad2deg(np.arctan2((dst[1] - src[1]), (dst[0] - src[0])))


def main():

    # %% init
    nDigits = 3
    i_start = 1
    i_end = 40
    N = i_end - i_start

    use_ransac = False
    save = True

    pwd_current = os.path.join(os.sep, "media", "taeke", "backup", "thesis_data",
                               "detect_truss")
    dataset = 'depth_blue'  # "drawing" #  "empty" # "artificial" # 'failures'#

    pwd_data = os.path.join(pwd_current, "data", dataset)
    pwd_results = os.path.join(pwd_current, "results", dataset, "06_peduncle")

    make_dirs(pwd_results)

    brightness = 0.85

    for count, i_tomato in enumerate(range(i_start, i_end)):  # 10, 11
        print("Analyzing image %d out of %d" % (i_tomato, N))

        tomato_name = str(i_tomato).zfill(nDigits)
        file_name = tomato_name + ".png"

        img_rgb = load_rgb(pwd_data, file_name, horizontal=True)

        image = ProcessImage(use_truss=True,
                             name=tomato_name,
                             pwd=pwd_results,
                             save=False)

        image.add_image(img_rgb)

        image.color_space()
        image.segment_image()
        image.filter_image()
        image.rotate_cut_img()
        image.detect_tomatoes()

        segment_img = image.get_segmented_image(local=True)
        peduncle_img = image.get_peduncle_image(local=True)
        segment_img_bright = change_brightness(segment_img, brightness)

        # skeleton_img, branch_data, coord_junc, coord_end = detect_peduncle(peduncle_img,
        #                                                                    use_ransac=use_ransac,
        #                                                                    bg_img=segment_img_bright,
        #                                                                    save=True,
        #                                                                    name=tomato_name,
        #                                                                    pwd=pwd_results)
        timeout = 1000
        skeleton_img = skeletonize_img(peduncle_img)

        if save:
            visualize_skeleton(segment_img_bright.copy(), skeleton_img, name=tomato_name + "_01", pwd=pwd_results)

        skeleton_img = threshold_branch_length(skeleton_img, 20.0)
        graph, pixel_coordinates, degree_image  = skan.skeleton_to_csgraph(skeleton_img)

        # compute paths
        dist, pred = csgraph.shortest_path(graph, return_predecessors=True)

        # all paths starting at an endpoint
        end_coords = np.argwhere(degree_image == 1)
        junc_coords = np.argwhere(degree_image == 3)

        end_nodes = coords_to_nodes(pixel_coordinates, end_coords)
        junc_nodes = coords_to_nodes(pixel_coordinates, junc_coords)

        if save:
            visualize_skeleton(segment_img_bright.copy(), skeleton_img, coord_junc=junc_coords[:, [1, 0]],
                           coord_end=end_coords[:, [1, 0]], name=tomato_name + "_02", pwd=pwd_results)


        # reserve memory
        n = len(end_coords)
        length_mat = np.full([n, n], np.nan)
        path_mat = np.full([n, n], np.nan)
        reward_mat = np.full([n, n], np.nan)


        for i, from_node in enumerate(end_nodes):
            for j, to_node in enumerate(end_nodes):

                # cstree = csgraph.reconstruct_path(csgraph=skeleton.graph, predecessors=pred[from_node], directed=False)
                # path, length = get_path(from_node, to_node, pred, dist)
                paths = []
                lengths = []
                path = []
                length = 0
                count = 0
                angle = None
                junc_end_coord = pixel_coordinates[from_node]
                init_junc_end_coord = junc_end_coord

                while (to_node != -9999) and (count < timeout):
                    if (to_node == junc_nodes).any() or np.any(to_node == end_nodes).any():
                        new_junc_end_coord = pixel_coordinates[to_node]
                        angle_new = node_coord_angle(junc_end_coord, new_junc_end_coord)

                        if angle is None:
                            diff = 0
                        else:
                            diff = abs(angle - angle_new)

                        if diff < 45:
                            # walk over branch
                            angle_total = node_coord_angle(init_junc_end_coord, new_junc_end_coord)  # angle_new
                            length += dist[(from_node, to_node)]

                        else:
                            # reset path
                            print 'Reset! difference is:', diff
                            junc_end_coord = pixel_coordinates[from_node]
                            init_junc_end_coord = junc_end_coord
                            paths.append(path)
                            lengths.append(length)
                            path = []
                            angle_total = None
                            length = 0

                        angle = angle_total

                    path.append(to_node)
                    count += 1
                    to_node = pred[(from_node, to_node)]

                paths.append(path)
                lengths.append(length)

                if len(paths) > 2:
                    ii = np.argmax(np.array(lengths))
                    length = lengths[ii]
                    length_mat[i, j] = length
                    print length
                    # path_mat[i, j] = paths[ii]
                else:
                    length_mat[i, j] = 0
                    # path_mat[i, j] = []
                    #
                # if len(path) > 3:
                #     row, col = get_path_coordinates(path, pixel_coordinates)
                #     param, cov = optimize.curve_fit(func, col, row)
                #     error = get_fit_error(row, col, param)
                #
                #     curvature = curvature_splines(col, y=row, error=0.25)
                #     plt.figure()
                #     plt.plot(curvature)
                #     print param
                #     fig = show_path(path, pixel_coordinates, skeleton_img.shape, param=param, show=False)
                # else:
                #     error = 9999

                # reward = length**1.2/error
                # print "error", error
                # print "reward", reward
                # error_mat[i, j] = error
                # reward_mat[i, j] = reward

        i = np.where(np.isinf(length_mat), -np.Inf, length_mat).argmax()
        # i = np.where(np.isinf(reward_mat), -np.Inf, reward_mat).argmax()
        from_i, to_i = np.unravel_index(i, reward_mat.shape)
        from_coord = end_coords[from_i]
        to_coord = end_coords[to_i]

        from_node = np.argmin(np.sum((pixel_coordinates - from_coord) ** 2, axis=1))
        to_node = np.argmin(np.sum((pixel_coordinates - to_coord) ** 2, axis=1))

        path, length = get_path(from_node, to_node, pred, dist)
        path_img = show_path(path, pixel_coordinates, skeleton_img.shape)

        visualize_skeleton(segment_img_bright.copy(), path_img, coord_junc=junc_coords[:, [1, 0]],
                           coord_end=end_coords[:, [1, 0]], name=tomato_name + "_03",
                           pwd=pwd_results)

    name_space = 'peduncle'
    plot_timer(Timer.timers[name_space].copy(), N=N, threshold=0.1, pwd=pwd_results, title='time')
    print(Counter.counters[name_space].copy())


if __name__ == '__main__':
    main()
