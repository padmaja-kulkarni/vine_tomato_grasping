from util import remove_blobs, bin2img, img2bin
from skimage.morphology import skeletonize

import numpy as np
import cv2

from skimage.morphology import skeletonize
import skan
import scipy.sparse.csgraph as csgraph
import scipy.optimize as optimize
from matplotlib import pyplot as plt

from util import add_circles, add_arrows
from util import save_img, save_fig
from util import remove_blobs, bin2img, img2bin


def set_detect_peduncle_settings(branch_length_min_px=15,
                                 branch_length_min_mm=10):
    settings = {'branch_length_min_px': branch_length_min_px,
                'branch_length_min_mm': branch_length_min_mm}
    return settings


def get_locations_on_mask(mask, coords, allowable_distance=1):
    mask_coords = np.argwhere(mask)
    coord_keep = []

    for coord in coords:
        dist = np.sqrt(np.sum(np.power(mask_coords - coord[[1, 0]], 2), 1))
        if np.amin(dist) < allowable_distance:
            coord_keep.append(coord)

    return np.array(coord_keep) # [:, [0, 1]]

def update_skeleton(skeleton_img, skeleton, i_remove):
    dtype = skeleton_img.dtype
    max_value = np.iinfo(dtype).max

    skeleton_prune_img = skeleton_img.copy()

    for i in i_remove:
        px_coords = skeleton.path_coordinates(i).astype(int)
        rows = px_coords[:, 0]
        cols = px_coords[:, 1]
        skeleton_prune_img[rows, cols] = 0

        # for px_coord in px_coords:
        #     skeleton_prune_img[px_coord[0], px_coord[1]] = 0

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
    i_remove = b_remove.to_numpy().nonzero()[0] # np.argwhere(b_remove)[:, 0]

    return update_skeleton(skeleton_img, skeleton, i_remove)


def get_node_coord(skeleton_img):
    if np.all(skeleton_img == 0):
        return None, None

    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)

    # get all node IDs
    src_node_id = np.unique(branch_data['node-id-src'].values)
    dst_node_id = np.unique(branch_data['node-id-dst'].values)
    all_node_id = np.unique(np.append(src_node_id, dst_node_id))

    end_node_index = skeleton.degrees[all_node_id] == 1

    end_node_id = all_node_id[end_node_index]
    junc_node_id = np.setdiff1d(all_node_id, end_node_id)

    # swap cols
    end_node_coord = skeleton.coordinates[end_node_id][:, [1, 0]]
    junc_node_coord = skeleton.coordinates[junc_node_id][:, [1, 0]]
    return junc_node_coord, end_node_coord


def distance(coords, coord):
    ''' euclidian distance between coordinates two numpy array'''
    return np.sum((coords - coord) ** 2, axis=1)

def coords_to_nodes(pixel_coordinates, coords):
    nodes = []
    for coord in coords:
        nodes.append(np.argmin(distance(pixel_coordinates, coord)))

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
    path_coordinates = np.rint(pixel_coordinates[path]).astype(int)
    col = path_coordinates[:, 1]
    row = path_coordinates[:, 0]
    return row, col


def show_path(path, pixel_coordinates, shape, subpath=None, show=False):
    img = np.zeros(shape)
    if len(path) != 0:
        row, col = get_path_coordinates(path, pixel_coordinates)
        img[row, col] = 255
    else:
        img[0, 0] = 255

    if subpath is not None:
        row, col = get_path_coordinates(subpath, pixel_coordinates)
        img[row, col] = 125

    if show:
        plt.figure()
        plt.imshow(img)
        plt.show()

    return img.astype(np.uint8)


def visualize_skeleton(img, skeleton_img, skeletonize=False, coord_junc=None, junc_nodes=None, end_nodes=None,
                       coord_end=None, branch_data=None, name="", pwd=None):
    junc_color = (100, 0, 200)
    end_color = (200, 0, 0)
    pend_color = (0, 150, 30)

    if skeletonize:
        skeleton_img = skeletonize_img(skeleton_img)

    contours, hierarchy = cv2.findContours(skeleton_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    img = cv2.drawContours(img.copy(), contours, -1, pend_color, 2)

    if len(np.argwhere(skeleton_img)) > 2:

        if (coord_junc is None) and (coord_end is None):
            coord_junc, coord_end = get_node_coord(skeleton_img)

        elif coord_junc is None:
            coord_junc, _ = get_node_coord(skeleton_img)

        elif coord_end is None:
            _, coord_end = get_node_coord(skeleton_img)

        add_circles(img, coord_junc, color=junc_color, thickness=-1, radii=3)
        add_circles(img, coord_end, color=end_color, thickness=-1, radii=3)

    if branch_data:
        branch_center = {}
        branch_angle = {}
        for branch_type in branch_data:
            branch_center[branch_type] = []
            branch_angle[branch_type] = []
            for branch in branch_data[branch_type]:
                branch_center[branch_type].append(branch['center_node_coord'])
                branch_angle[branch_type].append(branch['angle'])

        add_arrows(img, branch_center['junction-junction'], branch_angle['junction-junction'],
                   l=20, color=junc_color, thickness=2, tip_length=0.5, is_rad=False)
        add_arrows(img, branch_center['junction-endpoint'], branch_angle['junction-endpoint'],
                   l=20, color=end_color, thickness=2, tip_length=0.5, is_rad=False)

    if (junc_nodes is not None) or (end_nodes is not None):
        plt.figure()
        plt.imshow(img)
        if junc_nodes is not None:
            for junc_node, coord in zip(junc_nodes, coord_junc):
                plt.text(coord[0], coord[1], str(junc_node))

        if end_nodes is not None:
            for end_node, coord in zip(end_nodes, coord_end):
                plt.text(coord[0], coord[1], str(end_node))
        plt.show()

    if pwd:
        save_img(img, pwd, name)
    return img

def node_coord_angle(src, dst):
    return np.rad2deg(np.arctan2((dst[0] - src[0]), (dst[1] - src[1])))

def skeletonize_img(img):
    return bin2img(skeletonize(img2bin(img)))


def find_path(dist, pred, junc_nodes, end_nodes, pixel_coordinates, shape=None, timeout=1000):
    # initialize
    best_path = []
    best_length = 0

    for start_node in end_nodes:  # [931]: #
        for end_node in end_nodes:  # [94]: #

            count = 0
            angle_total = None
            from_node = start_node
            coord = pixel_coordinates[from_node]
            init_coord = coord
            length = 0

            path = []
            subpath = []
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

                    # show_path(path, pixel_coordinates, shape, subpath=subpath, show=True)
                    if diff < 45:
                        angle_total = node_coord_angle(init_coord, new_coord)  # angle_new

                        path.extend(subpath)
                        branch_data.append(subpath)
                        if len(subpath) == 0:
                            print subpath

                        subpath = []

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
                        subpath = []
                        length = 0
                        angle_total = node_coord_angle(init_coord, new_coord)

                    coord = new_coord

            if len(path) > 1:
                length = dist[(path[0], path[-1])]

            if length >= best_length:
                best_path = path
                best_length = length
                best_branch_data = branch_data

    return best_path, best_length, best_branch_data

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
        coords = pixel_coordinates[branch] ## skeleton.path_coordinates(i_row)
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

def detect_peduncle(peduncle_img, settings=None, px_per_mm=None, bg_img=None, save=False, name="", pwd=""):

    if settings is None:
        settings = set_detect_peduncle_settings()

    if bg_img is None:
        bg_img = peduncle_img

    if px_per_mm:
        branch_length_min_px = px_per_mm * settings['branch_length_min_mm']
    else:
        branch_length_min_px = settings['branch_length_min_px']

    skeleton_img = skeletonize_img(peduncle_img)
    junc_coords, end_coords = get_node_coord(skeleton_img)

    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, coord_junc=junc_coords, coord_end=end_coords,
                           name=name + "_01", pwd=pwd)

    skeleton_img = threshold_branch_length(skeleton_img, branch_length_min_px)
    junc_coords, end_coords = get_node_coord(skeleton_img)

    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, coord_junc=junc_coords, coord_end=end_coords,
                           name=name + "_02", pwd=pwd)

    graph, pixel_coordinates, degree_image = skan.skeleton_to_csgraph(skeleton_img, unique_junctions=True)
    dist, pred = csgraph.shortest_path(graph, directed=False, return_predecessors=True)

    end_nodes = coords_to_nodes(pixel_coordinates, end_coords[:, [1, 0]])
    junc_nodes = coords_to_nodes(pixel_coordinates, junc_coords[:, [1, 0]])

    path, length, branch_data = find_path(dist, pred, junc_nodes, end_nodes, pixel_coordinates, shape=skeleton_img.shape)
    branch_data = get_branch_center(branch_data, dist, pixel_coordinates, skeleton_img)

    path_img = show_path(path, pixel_coordinates, skeleton_img.shape)
    junc_coords = get_locations_on_mask(path_img, junc_coords)
    end_coords = get_locations_on_mask(path_img, end_coords)


    # end_coords = np.array([branch_data['junction-endpoint'][0]['src_node_coord'],
    #                        branch_data['junction-endpoint'][-1]['dst_node_coord']])
    end_coords = np.array([pixel_coordinates[path[0]][[1, 0]], pixel_coordinates[path[-1]][[1, 0]]])

    for end_coord in end_coords:
        dst = distance(junc_coords, end_coord)
        mask = dst > 0.1
        junc_coords = junc_coords[mask]

    if save:
        visualize_skeleton(bg_img.copy(), path_img, coord_junc=junc_coords, # junc_nodes=junc_nodes, end_nodes=end_nodes,
                           coord_end=end_coords, name=name + "_03", pwd=pwd)

    if save:
        visualize_skeleton(bg_img.copy(), path_img, coord_junc=junc_coords,
                           branch_data=branch_data,
                           coord_end=end_coords, name=name + "_04", pwd=pwd)

    return path_img, branch_data, junc_coords, end_coords