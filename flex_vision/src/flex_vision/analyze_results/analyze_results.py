# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 14:22:48 2020

@author: taeke
"""

import os
import json
import numpy as np
from flex_vision.utils.util import load_rgb, plot_features, plot_error, make_dirs, plot_features_result
from flex_vision.detect_truss.detect_tomato import compute_com
from sklearn.metrics.pairwise import euclidean_distances as euclidean_distance_matrix


def remove_none_from_list(lst_none):
    return [x for x in lst_none if x is not None]


def euclidean_distances(l1, l2, factor=None):
    """
    compute the euclidean distance between elements in list1 and list2
    """
    distances = []
    for p1, p2 in zip(l1, l2):
        distances.append(euclidean_distance(p1, p2, factor))

    return distances


def euclidean_distance(p1, p2, factor=None):
    """
    compute the euclidean distance between point_1 and point_2
    """
    if p2 == [] or p1 == []:
        return None

    if factor is None:
        return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    else:
        return factor*((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5

def index_true_positives(lbl_centers, res_centers, dist_tresh):
    """
    For each labeled center find the closest prediction. Is a distnace thershold is surpassed, it is a false negative.
    - lbl_centers: centers ground truth
    - res_centers: predicted centers
    - dist_tresh: the maximum allowable distance, for which the predictions may still be labeled as a true possitive
    """

    dist = euclidean_distance_matrix(lbl_centers, res_centers)

    # if the distance is too large, it is a false positive!
    i_keep = np.amin(dist, axis=1) < dist_tresh
    true_pos = np.argmin(dist, axis=1)[i_keep].tolist()
    false_pos = list(set(range(len(res_centers))) - set(true_pos))
    false_neg = np.argwhere(np.logical_not(i_keep))[:, 0].tolist()

    # determine duplicates, one of these is a false negative!
    duplicates = set([duplicates for duplicates in true_pos if true_pos.count(duplicates) > 1])

    rows_remove = []
    for duplicate in duplicates:
        col = duplicate

        # find the rows corresponding to duplicate, cannot
        rows = [i for i, j in enumerate(true_pos) if j == duplicate]
        i_keep = np.argmin(dist[rows, col])
        i_remove = list(set(range(len(rows))) - set([i_keep]))

        rows_remove = list(np.array(rows)[i_remove])

        for row_remove in sorted(rows_remove, reverse=True):
            true_pos.pop(row_remove)

        false_neg.extend(rows_remove)

    true_pos_lbl = list(set(range(len(lbl_centers))) - set(false_neg))
    true_pos_res = true_pos

    return true_pos_res, true_pos_lbl, false_pos, false_neg


def main():
    i_start = 1
    i_end = 85
    save_results = False
    N = i_end - i_start

    drive = "backup"  # "UBUNTU 16_0"  #
    # pwd_root = os.path.join(os.sep, 'home', 'taeke', 'Documents', "images")
    pwd_root = os.path.join(os.sep, "media", "taeke", "backup", "thesis_data", "detect_truss")
    pwd_lbl = os.path.join(pwd_root, "data", "lidl")
    pwd_res = os.path.join(pwd_root, "results", "lidl", 'json')
    if save_results:
        pwd_store = os.path.join(pwd_root, "results", 'final')
        make_dirs(pwd_store)
    else:
        pwd_store = None

    tomato_error_all = {}
    junction_error_all = {}

    dist_thresh_tomato = 15  # [mm]
    dist_thresh_peduncle = 10  # [mm]

    for count, i_truss in enumerate(range(i_start, i_end)):
        print("Analyzing image %d out of %d" % (count + 1, N))

        truss_name = str(i_truss).zfill(3)

        if i_truss > 49:
            extension = "_rgb"
        else:
            extension = ""

        file_lbl = os.path.join(pwd_lbl, truss_name + extension + '.json')
        file_inf = os.path.join(pwd_lbl, truss_name + '_info.json')
        file_res = os.path.join(pwd_res, truss_name + '.json')

        # load data
        img_rgb = load_rgb(truss_name + '_rgb.png', pwd_lbl, horizontal=False)

        if not os.path.exists(file_lbl):
            print('Labels do not exist for image: ' + truss_name + ' skipping this file')
            continue

        if not os.path.exists(file_inf):
            print('Info does not exist for image: ' + truss_name + 'continueing without info. THIS MAY YIELD '
                                                                         'STRANGE RESULTS!')
            use_mm = False
        else:
            use_mm = True

        with open(file_lbl, "r") as read_file:
            data_lbl = json.load(read_file)

        data_lbl.pop('imageData', None)
        shapes = data_lbl['shapes']

        tomato_lbl = {'radii': [], 'centers': []}
        peduncle_lbl = {'junctions': [], 'ends': []}

        for shape in shapes:

            label = shape['label']
            shape_type = shape['shape_type']
            if label == 'tomato':
                if shape_type != 'circle':
                    print("I do not know what to do with ", label, " of shape type ", shape_type)

                else:
                    points = shape['points']
                    center = points[0]
                    radius = euclidean_distance(points[0], points[1])

                    tomato_lbl['centers'].append(center)
                    tomato_lbl['radii'].append(radius)

            elif label == 'junction':
                if shape_type != 'point':
                    print("I do not know what to do with ", label, " of shape type ", shape_type)

                else:
                    point = shape['points'][0]
                    peduncle_lbl['junctions'].append(point)

            elif label == 'end_point' or label == 'end':
                if shape_type != 'point':
                    print("I do not know what to do with ", label, " of shape type ", shape_type)

                point = shape['points'][0]
                peduncle_lbl['ends'].append(point)

            else:
                print "i do not know what to do with ", label

        if use_mm:
            with open(file_inf, "r") as read_file:
                data_inf = json.load(read_file)
            unit = '[mm]'
            px_per_mm = data_inf['px_per_mm']

        else:
            unit = '[px]'
            px_per_mm = 1

        # compute com
        tomato_lbl['com'] = compute_com(tomato_lbl['centers'], tomato_lbl['radii'])

        if save_results:
            plot_features(img_rgb, tomato=tomato_lbl, pwd=pwd_store, file_name=truss_name + '_tom_lbl')
            plot_features(img_rgb, peduncle=peduncle_lbl, pwd=pwd_store, file_name=truss_name + '_pend_lbl')
            plot_features(img_rgb, tomato=tomato_lbl, peduncle=peduncle_lbl, pwd=pwd_store, file_name=truss_name + '_lbl')

        with open(file_res, "r") as read_file:
            data_results = json.load(read_file)

        img_res = img_rgb.copy()
        tomato_res = data_results['tomato']
        peduncle_res = data_results['peduncle']
        grasp_res = data_results['grasp_location']

        i_true_pos_res, i_true_pos_lbl, i_false_pos, i_false_neg = index_true_positives(tomato_lbl['centers'],
                                                                                        tomato_res['centers'],
                                                                                        dist_thresh_tomato * px_per_mm)

        tomato_pred = {}
        tomato_pred['true_pos'] = {}
        tomato_pred['false_pos'] = {}
        tomato_pred['com'] = tomato_res['com']

        tomato_actual = {}
        tomato_actual['true_pos'] = {}
        tomato_actual['false_neg'] = {}
        tomato_actual['com'] = tomato_lbl['com']

        for key in ['centers', 'radii']:
            tomato_pred['true_pos'][key] = np.array(tomato_res[key])[i_true_pos_res].tolist()
            tomato_pred['false_pos'][key] = np.array(tomato_res[key])[i_false_pos].tolist()

            tomato_actual['true_pos'][key] = np.array(tomato_lbl[key])[i_true_pos_lbl].tolist()
            tomato_actual['false_neg'][key] = np.array(tomato_lbl[key])[i_false_neg].tolist()

        n_true_pos = len(i_true_pos_res)
        n_false_pos = len(i_false_pos)
        n_labeled_pos = len(tomato_lbl['centers'])
        n_predict_pos = len(tomato_pred['true_pos']['centers'])

        com_error = euclidean_distance(tomato_lbl['com'][0], tomato_res['com']) / px_per_mm
        centers_error = euclidean_distances(tomato_actual['true_pos']['centers'], tomato_pred['true_pos']['centers'], factor=1/px_per_mm)
        radii_error = [abs(r1-r2)/px_per_mm for r1, r2 in zip(tomato_actual['true_pos']['radii'], tomato_pred['true_pos']['radii'])]

        # compute error
        tomato_error = {'radii': radii_error, 'centers': centers_error, 'com': com_error, 'n_true_pos': n_true_pos,
                        'n_false_pos': n_false_pos, 'n_labeled_pos': n_labeled_pos, 'n_predict_pos': n_predict_pos}

        # plot
        if save_results:
            plot_features_result(img_res, tomato_pred=tomato_pred, name=truss_name + '_temp')
            plot_error(tomato_pred=tomato_pred,  # centers, com,
                       tomato_act=tomato_actual,
                       error=tomato_error,  # center radii and com
                       pwd=pwd_store,
                       name=truss_name + '_tom_error',
                       use_mm=use_mm)

        # store
        tomato_error_all[truss_name] = tomato_error

        i_true_pos_res, i_true_pos_lbl, i_false_pos, i_false_neg = index_true_positives(peduncle_lbl['junctions'],
                                                                                        peduncle_res['junctions'],
                                                                                        dist_thresh_peduncle * px_per_mm)

        junction_pred = {}
        junction_pred['true_pos'] = {}
        junction_pred['false_pos'] = {}

        junction_actual = {}
        junction_actual['true_pos'] = {}
        junction_actual['false_neg'] = {}

        for key in ['centers']:
            junction_pred['true_pos'][key] = np.array(peduncle_res['junctions'])[i_true_pos_res].tolist()
            junction_pred['false_pos'][key] = np.array(peduncle_res['junctions'])[i_false_pos].tolist()

            junction_actual['true_pos'][key] = np.array(peduncle_lbl['junctions'])[i_true_pos_lbl].tolist()
            junction_actual['false_neg'][key] = np.array(peduncle_lbl['junctions'])[i_false_neg].tolist()

        n_true_pos = len(i_true_pos_res)
        n_false_pos = len(i_false_pos)
        n_labeled_pos = len(peduncle_lbl['junctions'])
        n_predict_pos = len(peduncle_res['junctions'])

        center_error = euclidean_distances(junction_actual['true_pos']['centers'], junction_pred['true_pos']['centers'],
                                            factor=1.0/px_per_mm)

        junctions_error = {'centers': center_error, 'true_pos': n_true_pos, 'false_pos': n_false_pos, 'labeled_pos': n_labeled_pos,
                           'predict_pos': n_predict_pos}

        # plot
        if save_results:
            plot_features_result(img_res, peduncle=junction_pred)  # grasp = grasp_ress
            plot_error(tomato_pred=junction_pred,  # centers, com,
                       tomato_act=junction_actual,
                       error=junctions_error,
                       pwd=pwd_store,
                       name=truss_name + '_pend_error',
                       use_mm=use_mm)

        # store
        junction_error_all[truss_name] = junctions_error

    tomato_error_centers = []
    tomato_error_radii = []
    tomato_error_com = []
    n_true_pos = 0
    n_false_pos = 0
    n_labeled_pos = 0
    n_predict_pos = 0

    # not in order be default!
    all_ids = junction_error_all.keys()
    all_ids.sort()
    for id in all_ids:
        tomato_error_centers.extend(tomato_error_all[id]['centers'])
        tomato_error_radii.extend(tomato_error_all[id]['radii'])
        tomato_error_com.append(tomato_error_all[id]['com'])
        n_true_pos += tomato_error_all[id]['n_true_pos']
        n_false_pos += tomato_error_all[id]['n_false_pos']
        n_labeled_pos += tomato_error_all[id]['n_labeled_pos']
        n_predict_pos += tomato_error_all[id]['n_predict_pos']

        if tomato_error_all[id]['n_labeled_pos'] - tomato_error_all[id]['n_true_pos'] > 0:
            print id

    error_tomato_center_mean = np.mean(tomato_error_centers)
    error_tomato_center_std = np.std(tomato_error_centers)

    error_tomato_radius_mean = np.mean(tomato_error_radii)
    error_tomato_radius_std = np.std(tomato_error_radii)

    error_com_center_mean = np.mean(tomato_error_com)
    error_com_center_std = np.std(tomato_error_com)

    true_pos_perc = int(round(float(n_true_pos) / float(n_labeled_pos) * 100))
    false_pos_perc = int(round(float(n_false_pos) / float(n_predict_pos) * 100))

    print 'Tomato center error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_tomato_center_mean,
                                                                                        std=error_tomato_center_std,
                                                                                        n=n_predict_pos, u=unit)
    print 'Tomato radius error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_tomato_radius_mean,
                                                                                        std=error_tomato_radius_std,
                                                                                        n=n_predict_pos, u=unit)
    print 'com error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_com_center_mean,
                                                                              std=error_com_center_std, n=n_predict_pos,
                                                                              u=unit)

    print 'True positive: {true_pos:d} out of {n_tomatoes:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos,
                                                                                          n_tomatoes=n_labeled_pos,
                                                                                          true_pos_perc=true_pos_perc)
    print 'False positive: {false_pos:d} out of {n_tomatoes:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos,
                                                                                             n_tomatoes=n_predict_pos,
                                                                                             false_pos_perc=false_pos_perc)

    cases = ['all', 'center', 'off-center']

    # Junctions https://stackoverflow.com/questions/22307628/python-how-to-extend-the-content-of-a-list-store-in-a-dict
    junction_error_centers = {k:[] for k in cases}
    junction_error_radii = {k:[] for k in cases}
    n_true_pos = dict.fromkeys(cases, 0)
    n_false_pos = dict.fromkeys(cases, 0)
    n_labeled_pos = dict.fromkeys(cases, 0)
    n_predict_pos = dict.fromkeys(cases, 0)

    for id in all_ids:

        id_type = ((int(id) - 1) % 7 + 1)
        if id_type in [1, 2, 3]:
            case = 'center'
        elif id_type in [4, 5, 6, 7]:
            case = 'off-center'
        else:
            print '?'

        for key in ['all', case]:
            junction_error_centers[key].extend(junction_error_all[id]['centers'])
            n_true_pos[key] += junction_error_all[id]['true_pos']
            n_false_pos[key] += junction_error_all[id]['false_pos']
            n_labeled_pos[key] += junction_error_all[id]['labeled_pos']
            n_predict_pos[key] += junction_error_all[id]['predict_pos']

    # mean and std without None
    for key in junction_error_centers:

        if n_labeled_pos[key] > 0:
            junction_error_centers_key = remove_none_from_list(junction_error_centers[key])
            error_juncion_center_mean = np.mean(junction_error_centers_key)
            error_junction_center_std = np.std(junction_error_centers_key)


            true_pos_perc = int(round(float(n_true_pos[key]) / float(n_labeled_pos[key]) * 100))
            false_pos_perc = int(round(float(n_false_pos[key]) / float(n_predict_pos[key]) * 100))
            print '===', key, '==='

            print 'Junction center error: {mean:.2f} {u:s} +- {std:.2f} {u:s} (n = {n:d})'.format(mean=error_juncion_center_mean,
                                                                                                  std=error_junction_center_std,
                                                                                                  n=n_true_pos[key], u=unit)

            print 'True positive: {true_pos:d} out of {n_junc_actual:d} ({true_pos_perc:d}%)'.format(true_pos=n_true_pos[key],
                                                                                                     n_junc_actual=n_labeled_pos[key],
                                                                                                     true_pos_perc=true_pos_perc)
            print 'False positive: {false_pos:d} out of {n_junct_predict:d} ({false_pos_perc:d}%)'.format(false_pos=n_false_pos[key],
                                                                                                          n_junct_predict=n_predict_pos[key],
                                                                                                          false_pos_perc=false_pos_perc)

if __name__ == '__main__':
    main()