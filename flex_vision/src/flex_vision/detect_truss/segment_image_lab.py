import cv2
import numpy as np
from segment_image import normalize_image, label2img, a_hist
from sklearn.metrics.pairwise import euclidean_distances

def segment_truss(img_a, img_b, save="False", name="", pwd="", settings=None):
    if settings is None:
        settings = settings.segment_image()

    n = 3
    centers_prior_a = {'tomato': 1.0,
                       'peduncle': -1.0,
                       'background': -1.0}
    centers_prior_b = {'tomato': 1.0,
                       'peduncle': 1.0,
                       'background': -1.0}

    centers_prior = {'a': centers_prior_a, 'b': centers_prior_b}
    centers = {'a': centers_prior['a'].values(), 'b': centers_prior['b'].values()}

    b_min = np.min(img_b)
    b_max = np.max(img_b)
    img_b_norm = normalize_image(img_b)

    a_min = np.min(img_a)
    a_max = np.max(img_a)
    img_a_norm = normalize_image(img_a)
    centers, _ = k_means_lab(img_a_norm, img_b_norm, n, settings, centers=centers)  # centers
    labels = assign_labels_lab(img_a, img_b, centers)

    # determine which center corresponds to which segment
    center_array = np.column_stack((centers['a'], centers['b']))
    tomato_center = np.array((centers_prior['a']['tomato'], centers_prior['b']['tomato']), ndmin=2)
    peduncle_center = np.array((centers_prior['a']['peduncle'], centers_prior['b']['peduncle']), ndmin=2)

    lbl = {}
    lbl["tomato"] = np.argmin(euclidean_distances(center_array, tomato_center))
    lbl["peduncle"] = np.argmin(euclidean_distances(center_array, peduncle_center))
    lbl["background"] = list(set(range(0, n)) - set(lbl.values()))[0]

    # compute masks
    dim = img_a.shape
    tomato = label2img(labels, lbl["tomato"], dim)
    peduncle = label2img(labels, lbl["peduncle"], dim)
    background = label2img(labels, lbl["background"], dim)
    if save:
    #     both_hist(img_hue, img_a_norm, centers, lbl, a_bins=a_max - a_min + 1, pwd=pwd, name=name,
    #               hue_radius=settings['hue_radius'])
    #     hue_hist(img_hue, np.rad2deg(centers['hue']), lbl, name, pwd)
    #     if img_a is not None:
        a_hist(img_a_norm, centers['a'], lbl, bins=a_max - a_min + 1, name=name, pwd=pwd)
        a_hist(img_b_norm, centers['b'], lbl, bins=b_max - b_min + 1, name=name + '_b', pwd=pwd)

    return background, tomato, peduncle

def k_means_lab(img_a, img_b, n_clusters, settings, centers=None):
    # convert hue value to angles, and place on unit circle

    h, w = img_a.shape

    new_shape = (w / settings['f'], h / settings['f'])
    img_a = cv2.resize(img_a, new_shape, interpolation=cv2.INTER_NEAREST)
    img_b = cv2.resize(img_b, new_shape, interpolation=cv2.INTER_NEAREST)

    # angle = np.deg2rad(2 * np.float32(img_hue.flatten()))
    # hue_radius = settings['hue_radius']
    data = np.stack((0.5*img_a.flatten(), img_b.flatten()), axis=1)
    labels = None

    if centers is not None:
        labels = np.array(assign_labels_lab(img_a, img_b, centers), dtype=np.int32)
        flags = cv2.KMEANS_USE_INITIAL_LABELS
        attempts = 3
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
    centers_out['a'] = centers_xy[:, 0]
    centers_out['b'] = centers_xy[:, 1]
    return centers_out, labels

def assign_labels_lab(img_a, img_b, centers_dict):
    """Assign labels based on centers"""
    data_b = np.expand_dims(normalize_image(img_b).flatten(), axis=1)
    data = data_b
    center_b = centers_dict['b']
    centers = np.expand_dims(center_b, axis=1)

    data_a = np.expand_dims(normalize_image(img_a).flatten(), axis=1)
    data = np.append(data, data_a, axis=1)
    center_a = np.expand_dims(centers_dict['a'], axis=1)
    centers = np.append(centers, center_a, axis=1)

    dist = euclidean_distances(data, centers)
    labels = np.argmin(dist, axis=1)
    return labels