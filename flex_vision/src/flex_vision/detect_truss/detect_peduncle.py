# -*- coding: utf-8 -*-

## imports ##
import cv2
import numpy as np
import skan

# custom functions
from flex_vision.utils.util import add_circles, add_arrows
from flex_vision.utils.util import save_img, save_fig
from flex_vision.utils.util import remove_blobs, bin2img, img2bin

from skimage.morphology import skeletonize
from sklearn import linear_model

from flex_vision.utils.timer import Timer
from flex_vision.utils.counter import Counter
from matplotlib import pyplot as plt


def set_detect_peduncle_settings(branch_length_min_px=15,
                                 branch_length_min_mm=10):
    settings = {'branch_length_min_px': branch_length_min_px,
                'branch_length_min_mm': branch_length_min_mm}
    return settings


def get_node_id(branch_data, skeleton):
    src_node_id = np.unique(branch_data['node-id-src'].values)
    dst_node_id = np.unique(branch_data['node-id-dst'].values)
    all_node_id = np.unique(np.append(src_node_id, dst_node_id))

    # deg = skeleton.degrees[all_node_id]
    end_node_index = skeleton.degrees[all_node_id] == 1  # np.argwhere(deg == 1)[:, 0] # endpoint

    end_node_id = all_node_id[end_node_index]
    junc_node_id = np.setdiff1d(all_node_id, end_node_id)

    return junc_node_id, end_node_id


@Timer("get node coord", name_space='peduncle', append=False)
def get_node_coord(skeleton_img):
    if np.all(skeleton_img == 0):
        return None, None

    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)

    # get all node IDs
    junc_node_id, end_node_id = get_node_id(branch_data, skeleton)

    # swap cols
    end_node_coord = skeleton.coordinates[end_node_id][:, [1, 0]]
    junc_node_coord = skeleton.coordinates[junc_node_id][:, [1, 0]]

    return junc_node_coord, end_node_coord


@Timer('get locations on mask', name_space='peduncle', append=False)
def get_locations_on_mask(mask, locations):
    col, row = np.nonzero(mask)
    loc = np.transpose(np.matrix(np.vstack((row, col))))

    iKeep = []
    for i in range(locations.shape[0]):
        location = locations[i, :]
        # col, row = np.nonzero(skeleton)
        dist = np.sqrt(np.sum(np.power(loc - location, 2), 1))
        if np.amin(dist) < 5:
            iKeep.append(i)

    return locations[iKeep, :]


def get_attached_branches(branch_data, node_id):
    """ given a node id determined the attached branch ids """
    branch_id_src = (branch_data['node-id-src'] == node_id).to_numpy().nonzero()[0]
    branch_id_dst = (branch_data['node-id-dst'] == node_id).to_numpy().nonzero()[0]
    branch_id = np.append(branch_id_src, branch_id_dst)
    return branch_id


def get_attached_nodes(branch_data, branch_indexes):
    node_id_src = branch_data['node-id-src'][branch_indexes]
    node_id_dst = branch_data['node-id-dst'][branch_indexes]
    node_ids = np.unique(np.append(node_id_src, node_id_dst))
    return node_ids


def get_new_node(branch_data, branch_index, old_node):
    node_id_src = branch_data['node-id-src'][branch_index]
    node_id_dst = branch_data['node-id-dst'][branch_index]
    if node_id_src == old_node:
        return node_id_dst
    elif node_id_dst == old_node:
        return node_id_src
    else:
        print('Old node was not attached to this branch!')
        return None


def update_skeleton(skeleton_img, skeleton, i_remove):
    dtype = skeleton_img.dtype
    max_value = np.iinfo(dtype).max

    skeleton_prune_img = skeleton_img.copy()

    for i in i_remove:
        px_coords = skeleton.path_coordinates(i).astype(int)

        for px_coord in px_coords:
            skeleton_prune_img[px_coord[0], px_coord[1]] = 0

    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = skeleton_prune_img
    img = cv2.dilate(skeleton_prune_img, kernel, iterations=1)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = remove_blobs(img)
    return bin2img(skeletonize(img / max_value))


def generate_skeleton_img(skeleton, i_keep, shape, dtype=np.uint8):
    max_value = np.iinfo(dtype).max
    skeleton_img = np.zeros(shape, dtype=dtype)

    for i in i_keep:
        px_coords = skeleton.path_coordinates(i).astype(int)

        for px_coord in px_coords:
            skeleton_img[px_coord[0], px_coord[1]] = max_value

    ## closing
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    img = skeleton_img
    img = cv2.dilate(skeleton_img, kernel, iterations=1)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    return bin2img(skeletonize(img2bin(img)))


@Timer("threshold branch length", name_space='peduncle', append=False)
def threshold_branch_length(skeleton_img, distance_threshold):
    skeleton = skan.Skeleton(skeleton_img)
    branch_data = skan.summarize(skeleton)

    tip_junction = branch_data['branch-type'] == 1

    b_remove = (branch_data['branch-distance'] < distance_threshold) & tip_junction
    i_remove = b_remove.to_numpy().nonzero()[0] # np.argwhere(b_remove)[:, 0]

    return update_skeleton(skeleton_img, skeleton, i_remove)


@Timer("filter branch length", name_space='peduncle', append=False)
def filter_branch_length(skeleton_img, branch_data=None, debug =False):
    skeleton = skan.Skeleton(skeleton_img)

    if branch_data is None:
        branch_data = skan.summarize(skeleton)

    junc_node_ids, start_node_ids = get_node_id(branch_data, skeleton)

    max_path = list()
    max_length = 0

    if debug:
        n_start_nodes = len(start_node_ids)
        n_junc_nodes = len(junc_node_ids)
        n_vertices = n_start_nodes + n_junc_nodes
        n_edges = len(branch_data.values)  # 2*(n - 1)
        n_loop = 1 + n_edges - n_vertices

        n_paths_min = n_start_nodes * (n_edges + 1)  # n_vertices * (1 + 0.5*n_vertices)# (n_vertices)*n_start_nodes
        n_paths_max = n_paths_min * 2 ** n_loop  # n_vertices * (1 + 2**n_junc_nodes)
        print '|E|: %d' % n_edges
        print '|V|: %d' % n_vertices
        print 'loops: %d' % n_loop
        print 'Expected function calls between  %d and %d' % (n_paths_min, n_paths_max)

    for node_id in start_node_ids:

        current_path = list()
        current_length = 0
        current_path, current_length = find_largest_branch(branch_data, skeleton,
                                                           node_id, node_id, current_path, current_length)

        if current_length > max_length:
            max_path = current_path
            max_length = current_length

    return generate_skeleton_img(skeleton, max_path, skeleton_img.shape), max_path


@Counter("find largest branch", name_space='peduncle')
def find_largest_branch(branch_data, skeleton, node_id_current, node_id_start, path, length,
                        angle=None, max_path=None, max_length=0, branch_visited=None, depth=1):

    if max_path is None:
        max_path = []
    if branch_visited is None:
        branch_visited = []

    branch_indexes = get_attached_branches(branch_data, node_id_current)
    branch_indexes = list(set(branch_indexes) - set(branch_visited))

    for branch_index in branch_indexes:
        path_new = path[:]
        branch_visited_new = branch_visited[:]

        branch_visited_new.append(branch_index)
        node_id_new = get_new_node(branch_data, branch_index, node_id_current)
        angle_new = node_id_angle(node_id_current, node_id_new, skeleton)
        length_new = length
        depth_new = depth + 1

        if angle is None:
            diff = 0
        else:
            diff = abs(angle - angle_new)

        if diff < 45:
            # walk over branch
            angle_total = node_id_angle(node_id_start, node_id_new, skeleton)  # angle_new
            path_new.append(branch_index)
            length_new += skeleton.distances[branch_index]

        else:
            # reset path
            node_id_start = node_id_current
            path_new = [branch_index]
            angle_total = angle_new
            length_new = skeleton.distances[branch_index]

        if depth_new < 12:
            max_path, max_length = find_largest_branch(branch_data, skeleton,
                                                       node_id_new,
                                                       node_id_start,
                                                       path_new,
                                                       length_new,
                                                       angle=angle_total,
                                                       max_path=max_path,
                                                       max_length=max_length,
                                                       branch_visited=branch_visited_new,
                                                       depth=depth_new)

        else:
            pass
            # print('Reached maximum search depth!')

            # store as largest branch if required
    if length > max_length:
        max_path = path
        max_length = length

    return max_path, max_length


def node_id_angle(node_1_id, node_2_id, skeleton):
    src = skeleton.coordinates[node_1_id][[1, 0]]
    dst = skeleton.coordinates[node_2_id][[1, 0]]
    angle = np.rad2deg(np.arctan2((dst[1] - src[1]), (dst[0] - src[0])))
    return angle


def node_coord_angle(src, dst):
    angle = np.rad2deg(np.arctan2((dst[1] - src[1]), (dst[0] - src[0])))
    return angle

def visualize_skeleton(img, skeleton_img, skeletonize=False, coord_junc=None, junc_nodes=None, end_nodes=None,
                       coord_end=None, branch_data=None, name="", pwd=None):
    junc_color = (100, 0, 200)
    end_color = (200, 0, 0)
    pend_color = (0, 150, 30)

    if skeletonize:
        skeleton_img = skeletonize_img(skeleton_img)

    contours, hierarchy = cv2.findContours(skeleton_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    cv2.drawContours(img, contours, -1, pend_color, 2)

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


@Timer("get center branch", name_space='peduncle', append=False)
def get_branch_center(branch_data, skeleton, skeleton_img):
    col, row = np.nonzero(skeleton_img)
    loc = np.transpose(np.matrix(np.vstack((row, col))))
    all_branch = {'junction-junction': [], 'junction-endpoint': []}

    for i_row, row in branch_data.iterrows(): # iterrows():

        # px_coords = skeleton.path_coordinates(i).astype(int)
        # new_branch = {'src_node_coord' : , 'dst_node_coord' :, 'center_node_coord':, 'angle': [], 'length': []}        

        dst_node_coord = np.array((row['coord-dst-{1}'], row['coord-dst-{0}']))
        src_node_coord = np.array((row['coord-src-{1}'], row['coord-src-{0}']))
        angle = node_coord_angle(src_node_coord, dst_node_coord)
        length = row['euclidean-distance']

        center = (dst_node_coord + src_node_coord) / 2
        dist = np.sqrt(np.sum(np.power(loc - center, 2), 1))
        i = np.argmin(dist)
        center_node_coord = [loc[i, 0], loc[i, 1]]

        # branch_index = branch_data.index[i]
        coords = skeleton.path_coordinates(i_row)
        new_branch = {'coords': np.column_stack((coords[:, 1], coords[:, 0])),
                      'src_node_coord': src_node_coord,
                      'dst_node_coord': dst_node_coord,
                      'center_node_coord': center_node_coord,
                      'angle': angle,
                      'length': length}

        if row['branch-type'] == 1:  # junction-to-endpoint
            all_branch['junction-endpoint'].append(new_branch)

        else:
            all_branch['junction-junction'].append(new_branch)

    return all_branch


@Timer("skeletonize image", name_space='peduncle', append=False)
def skeletonize_img(img):
    return bin2img(skeletonize(img2bin(img)))


@Timer("summarize image", name_space='peduncle', append=False)
def summarize_img(skeleton_img):
    # summarize skeleton
    skeleton = skan.Skeleton(img2bin(skeleton_img))
    branch_data = skan.summarize(skeleton)
    return skeleton, branch_data


def mad(data, axis=None):
    '''
    mean absolute deviation
    '''
    return np.mean(np.absolute(data - np.median(data, axis)), axis)


def fit_ransac(peduncle_img, bg_img=None, save=False, name="", pwd=""):
    # skeletonize peduncle segment
    h, w = peduncle_img.shape

    mask_index = np.argwhere(peduncle_img)
    x = mask_index[:, 1].reshape(-1, 1)  # x, spould be 2D
    y = mask_index[:, 0]  # y

    # Robustly fit linear model with RANSAC algorithm
    mad_y = mad(y)
    ransac = linear_model.RANSACRegressor(max_trials=500, residual_threshold=0.8*mad_y)
    ransac.fit(x, y)
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    i_remove = [y[outlier_mask], x[outlier_mask, 0]]
    peduncle_img[tuple(i_remove)] = 0

    #    skeleton_img = skeletonize_img(peduncle_img)
    #    skeleton = skan.Skeleton(skeleton_img)
    #
    #    # branch data
    #    for i, path in enumerate(skeleton.path_coordinates):
    #        px_coords = path.astype(int)

    if save:
        line_x = np.arange(x.min(), x.max())[:, np.newaxis]
        line_y = h - ransac.predict(line_x)

        fig = plt.figure()
        plt.scatter(x[inlier_mask], h - y[inlier_mask], color='yellowgreen', marker='.',
                    label='Inliers')
        plt.scatter(x[outlier_mask], h - y[outlier_mask], color='gold', marker='.',
                    label='Outliers')
        plt.plot(line_x, line_y, color='navy', linewidth=2, label='RANSAC regressor')
        plt.axis('equal')
        plt.legend(loc='lower right')
        plt.xlabel("Input")
        plt.ylabel("Response")

        if pwd is not None:
            save_fig(fig, pwd, name + "_00", title="RANSAC")
        plt.close(fig)

    return peduncle_img


def detect_peduncle(peduncle_img, settings=None, px_per_mm=None, bg_img=None,
                    use_ransac=False, save=False, name="", pwd=""):
    if settings is None:
        settings = set_detect_peduncle_settings()

    if bg_img is None:
        bg_img = peduncle_img

    if px_per_mm:
        branch_length_min_px = px_per_mm * settings['branch_length_min_mm']
    else:
        branch_length_min_px = settings['branch_length_min_px']

    if use_ransac:
        peduncle_img = fit_ransac(peduncle_img, bg_img=bg_img, save=save, name=name, pwd=pwd)

    # skeletonize peduncle segment
    skeleton_img = skeletonize_img(peduncle_img)

    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, name=name + "_01", pwd=pwd)

        # prune all smal branches
    skeleton_img = threshold_branch_length(skeleton_img, branch_length_min_px)

    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, name=name + "_02", pwd=pwd)

    skeleton, branch_data = summarize_img(skeleton_img)

    # get all node coordiantes
    all_juncions, _ = get_node_coord(skeleton_img)

    # determine main peduncle   
    skeleton_img, i_keep = filter_branch_length(skeleton_img)
    branch_data = branch_data.loc[i_keep]

    # skeleton = skan.Skeleton(skeleton_img)
    # branch_data = skan.summarize(skeleton)

    # get end points
    _, coord_end = get_node_coord(skeleton_img)

    # only select the junctions which lie on the main peduncle
    coord_junc = get_locations_on_mask(skeleton_img, all_juncions)

    # get the centers of the obtained branches
    branch_data = get_branch_center(branch_data, skeleton, skeleton_img)

    # branch_image = np.zeros(skeleton_img.shape, dtype=np.int8)
    # for branch in branch_data['junction-junction']:
    #     coords = branch['coords'].astype(int)
    #
    #     for coord in coords:
    #         branch_image[coord[1], coord[0]] = 255
    #
    # fig = plt.figure()
    # plt.imshow(branch_image)
    # plt.show()


    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, coord_junc=coord_junc,
                           coord_end=coord_end, name=name + "_03",
                           pwd=pwd)
    if save:
        visualize_skeleton(bg_img.copy(), skeleton_img, coord_junc=coord_junc,
                           branch_data=branch_data,
                           coord_end=coord_end, name=name + "_04", pwd=pwd)

    return skeleton_img, branch_data, coord_junc, coord_end


if __name__ == '__main__':
    print('No main!')
