# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 18:16:42 2020

@author: taeke
"""

import os
from detect_truss.util import make_dirs, load_rgb
from detect_truss.util import change_brightness, plot_timer

from detect_truss.ProcessImage import ProcessImage
from detect_truss.detect_peduncle import detect_peduncle

from detect_truss.timer import Timer
from detect_truss.counter import Counter

import numpy as np
from skimage.morphology import skeletonize
import skan
import scipy.sparse.csgraph as csgraph
from matplotlib import pyplot as plt

if __name__ == '__main__':

    # %% init
    nDigits = 3
    i_start = 1
    i_end = 2
    N = i_end - i_start

    use_ransac = False

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
        skeleton_img = skeletonize(peduncle_img/255)*255
        skel_graph = skan.skeleton_to_csgraph(skeleton_img)

        graph = skel_graph[0]
        pixel_coordinates = skel_graph[1]
        degree_image = skel_graph[2]

        # compute paths
        dist, pred = csgraph.shortest_path(graph, return_predecessors=True)

        # all paths starting at an endpoint
        end_nodes = np.argwhere(degree_image == 1)
        for end_node in end_nodes:
            from_node = np.argmin(np.sum((pixel_coordinates - end_node)**2, axis=1))
            to_node = np.where(np.isinf(dist[from_node]), -np.Inf, dist[from_node]).argmax()
            # from_node, to_node = np.unravel_index(i, dist.shape[end_node_i])
            # cstree = csgraph.reconstruct_path(csgraph=skeleton.graph, predecessors=pred[from_node], directed=False)
            path = []
            i = 0
            while (to_node != -9999) and (i < timeout):
                i += 1
                path.append(to_node)
                # print 'from', from_node, 'to', to_node
                to_node = pred[(from_node, to_node)]
            # path_i = predecessors[coord[1]]
            path_coordinates = np.rint(pixel_coordinates[path]).astype(int)
            # img = np.zeros(skeleton_img.shape)
            # img[path_coordinates[:, 0], path_coordinates[:, 1]] = 255
            # plt.imshow(img)
            # plt.show()


    name_space = 'peduncle'
    plot_timer(Timer.timers[name_space].copy(), N=N, threshold=0.1, pwd=pwd_results, title='time')
    print(Counter.counters[name_space].copy())
