#!/usr/bin/env python2
# -*- coding: utf-8 -*-


## imports ##
import os
import warnings

import numpy as np
import cv2
import json

from skimage.transform import rotate
from utils.transform import translation_rot2or

from image import Image, add, compute_bbox, image_rotate, image_crop, image_cut

from utils.timer import Timer

# custom functions
from util import add_border

import point2d
from point2d import Point2D
from transform import Transform
from util import add_circles, plot_features

from util import make_dirs
from util import save_img, save_fig, figure_to_image
from util import load_rgb
from util import stack_segments
from util import plot_timer, plot_grasp_location, plot_image
from util import change_brightness, plot_segments

from matplotlib import pyplot as plt

from filter_segments import filter_segments
from detect_peduncle_2 import detect_peduncle, visualize_skeleton, set_detect_peduncle_settings
from detect_tomato import detect_tomato, set_detect_tomato_settings
from compute_grasp import set_compute_grasp_settings
from segment_image import segment_truss, set_segment_image_settings

warnings.filterwarnings('error', category=FutureWarning)


def load_px_per_mm(pwd, img_id):
    pwd_info = os.path.join(pwd, img_id + '_info.json')

    if not os.path.exists(pwd_info):
        print('Info does not exist for image: ' + img_id + ' continueing without info')
        return None

    with open(pwd_info, "r") as read_file:
        data_inf = json.load(read_file)

    return data_inf['px_per_mm']


class ProcessImage(object):
    version = '0.1'

    # frame ids
    ORIGINAL_FRAME_ID = 'original'
    LOCAL_FRAME_ID = 'local'

    name_space = 'main'  # used for timer

    def __init__(self,
                 use_truss=True,
                 save=False,
                 pwd='',
                 name='tomato',
                 ext='pdf'
                 ):

        self.ext = ext
        self.save = save
        self.use_truss = use_truss
        self.pwd = pwd
        self.name = name

        self.scale = None
        self.image = None
        self.shape = None
        self.px_per_mm = None

        self.background = None
        self.tomato = None
        self.peduncle = None

        self.grasp_point = None
        self.grasp_angle_local = None
        self.grasp_angle_global = None

        # load settings
        settings = {}
        settings['segment_image'] = set_segment_image_settings()
        settings['detect_tomato'] = set_detect_tomato_settings()
        settings['detect_peduncle'] = set_detect_peduncle_settings()
        settings['compute_grasp'] = set_compute_grasp_settings()
        self.settings = settings

    def add_image(self, data, px_per_mm=None, name=None):
        image = Image(data)

        # TODO: rescaling is currentle not supported, would be interesting to reduce computing power
        # scale = image.rescale(self.width_desired)

        self.scale = 1.0
        self.image = image
        self.shape = data.shape[:2]
        self.px_per_mm = px_per_mm

        self.grasp_point = None
        self.grasp_angle_local = None
        self.grasp_angle_global = None

        if name is not None:
            self.name = name

    @Timer("color space", name_space)
    def color_space(self, compute_a=True):
        pwd = os.path.join(self.pwd, '01_color_space')
        self.image_hue = cv2.cvtColor(self.image.data, cv2.COLOR_RGB2HSV)[:, :, 0]
        if compute_a:
            self.image_a = cv2.cvtColor(self.image.data, cv2.COLOR_RGB2LAB)[:, :, 1]
        else:
            self.image_a = None

        if self.save:
            save_img(self.image_hue, pwd, self.name + '_h_raw', vmin=0, vmax=180)
            save_img(self.image_a, pwd, self.name + '_a_raw', vmin=0, vmax=255)

            save_img(self.image_hue, pwd, self.name + '_h', color_map='HSV', vmin=0, vmax=180)
            save_img(self.image_a, pwd, self.name + '_a', color_map='Lab')

    @Timer("segmentation", name_space)
    def segment_image(self, radius=None):
        """segment image based on hue and a color components.

        Keyword arguments:
        radius -- hue circle radius (see https://surfdrive.surf.nl/files/index.php/s/StoH7xA87zUxl79 page 16)
        """

        if radius is None:
            pwd = os.path.join(self.pwd, '02_segment')
        else:
            pwd = os.path.join(self.pwd, '02_segment', str(radius))
            self.settings['segment_image']['hue_radius'] = radius

        success = True
        background, tomato, peduncle = segment_truss(self.image_hue,
                                                     img_a=self.image_a,
                                                     settings=self.settings['segment_image'],
                                                     save=self.save,
                                                     pwd=pwd,
                                                     name=self.name)

        self.background = Image(background)
        self.tomato = Image(tomato)
        self.peduncle = Image(peduncle)

        if self.tomato.is_empty():
            warnings.warn("Segment truss: no pixel has been classified as tomato!")
            success = False

        if self.peduncle.is_empty():
            warnings.warn("Segment truss: no pixel has been classified as peduncle!")
            success = False

        if self.save:
            self.save_results(self.name, pwd=pwd)

        return success

    @Timer("filtering", name_space)
    def filter_image(self, folder_name=None):
        """ remove small pixel blobs from the determined segments

        Keyword arguments:
        folder_name -- name of folder where results are stored
        """
        pwd = os.path.join(self.pwd, '03_filter')
        if folder_name is not None:
            pwd = os.path.join(pwd, folder_name)

        tomato, peduncle, background = filter_segments(self.tomato.data,
                                                       self.peduncle.data,
                                                       self.background.data)

        self.tomato.data = tomato
        self.peduncle.data = peduncle
        self.background.data = background

        if self.save:
            self.save_results(self.name, pwd=pwd)

        if self.tomato.is_empty():
            warnings.warn("filter segments: no pixel has been classified as tomato")
            return False

        return True

    @Timer("cropping", name_space)
    def rotate_cut_img(self):
        """crop the image"""
        pwd = os.path.join(self.pwd, '04_crop')

        if self.peduncle.is_empty():
            warnings.warn("Cannot rotate based on peduncle, since it does not exist!")
            angle = 0
        else:
            angle = self.peduncle.compute_angle()  # [rad]

        tomato_rotate = image_rotate(self.tomato, -angle)
        peduncle_rotate = image_rotate(self.peduncle, -angle)
        truss_rotate = add(tomato_rotate, peduncle_rotate)

        if truss_rotate.is_empty():
            warnings.warn("Cannot crop based on truss segment, since it does not exist!")
            return False

        bbox = compute_bbox(truss_rotate.data)
        x = bbox[0]  # col
        y = bbox[1]  # row

        translation = np.array([[y, x]]).T
        self.transform = Transform(self.ORIGINAL_FRAME_ID,
                                   self.LOCAL_FRAME_ID,
                                   self.shape,
                                   angle=-angle,
                                   translation=translation)

        self.bbox = bbox
        self.angle = angle

        self.tomato_crop = tomato_rotate.copy().cut(self.bbox)
        self.peduncle_crop = peduncle_rotate.copy().cut(self.bbox)
        self.image_crop = self.image.copy().crop(-angle, bbox)
        self.truss_crop = truss_rotate.copy().cut(self.bbox)

        if self.save:
            self.save_results(self.name, pwd=pwd, local=True)

    @Timer("tomato detection", name_space)
    def detect_tomatoes(self):
        """detect tomatoes based on the truss segment"""
        pwd = os.path.join(self.pwd, '05_tomatoes')

        if self.peduncle.is_empty():
            return False

        if self.save:
            img_bg = self.image_crop.data
        else:
            img_bg = self.image_crop.data

        centers, radii, com = detect_tomato(self.truss_crop.data,
                                            self.settings['detect_tomato'],
                                            px_per_mm=self.px_per_mm,
                                            img_rgb=img_bg,
                                            save=self.save,
                                            pwd=pwd,
                                            name=self.name)

        # convert obtained coordinated to two-dimensional points linked to a coordinate frame
        self.radii = radii
        self.centers = point2d.from_coord_list(centers, self.LOCAL_FRAME_ID)
        self.com = Point2D(com, self.LOCAL_FRAME_ID)

        if self.com is None:
            return False
        else:
            return True

    @Timer("peduncle detection", name_space)
    def detect_peduncle(self):
        """Detect the peduncle and junctions"""
        pwd = os.path.join(self.pwd, '06_peduncle')
        success = True

        if self.save:
            img_bg = change_brightness(self.get_segmented_image(local=True), 0.85)  # self.image_crop.data #
        else:
            img_bg = self.image_crop.data

        mask, branch_data, junc_coords, end_coords = detect_peduncle(self.peduncle_crop.data,
                                                                     self.settings['detect_peduncle'],
                                                                     px_per_mm=self.px_per_mm,
                                                                     save=self.save,
                                                                     bg_img=img_bg,
                                                                     name=self.name,
                                                                     pwd=pwd)
        # convert to 2D points
        junction_points = point2d.from_coord_list(junc_coords, self.LOCAL_FRAME_ID)
        end_points = point2d.from_coord_list(end_coords, self.LOCAL_FRAME_ID)

        for branch_type in branch_data:
            for i, branch in enumerate(branch_data[branch_type]):
                for lbl in ['coords', 'src_node_coord', 'dst_node_coord', 'center_node_coord']:

                    if lbl == 'coords':
                        branch_data[branch_type][i][lbl] = point2d.from_coord_list(branch[lbl], self.LOCAL_FRAME_ID)
                    else:
                        branch_data[branch_type][i][lbl] = Point2D(branch[lbl], self.LOCAL_FRAME_ID)

        self.junction_points = junction_points
        self.end_points = end_points
        self.penduncle_main = mask
        self.branch_data = branch_data
        return success

    @Timer("detect grasp location", name_space)
    def detect_grasp_location(self):
        """Determine grasplocation based on peduncle, junction and com information"""
        pwd = os.path.join(self.pwd, '07_grasp')
        success = True

        settings = self.settings['compute_grasp']

        # set dimensions
        if self.px_per_mm is not None:
            minimum_grasp_length_px = self.px_per_mm * settings['grasp_length_min_mm']
        else:
            minimum_grasp_length_px = settings['grasp_length_min_px']

        points_keep = []
        branches_i = []
        for branch_i, branch in enumerate(self.branch_data['junction-junction']):
            if branch['length'] > minimum_grasp_length_px:

                src_node_dist = branch['src_node_coord'].dist(branch['coords'], self.transform)
                dst_node_dist = branch['dst_node_coord'].dist(branch['coords'], self.transform)
                is_true = np.logical_and((np.array(dst_node_dist) > 0.5 * minimum_grasp_length_px), (
                        np.array(src_node_dist) > 0.5 * minimum_grasp_length_px))

                branch_points_keep = np.array(branch['coords'])[is_true].tolist()
                points_keep.extend(branch_points_keep)
                branches_i.extend([branch_i] * len(branch_points_keep))

        if len(branches_i) > 0:
            i_grasp = np.argmin(self.com.dist(points_keep, self.transform))
            grasp_point = points_keep[i_grasp]
            branch_i = branches_i[i_grasp]

            grasp_angle_local = np.deg2rad(self.branch_data['junction-junction'][branch_i]['angle'])
            grasp_angle_global = -self.angle + grasp_angle_local

            self.grasp_point = grasp_point
            self.grasp_angle_local = grasp_angle_local
            self.grasp_angle_global = grasp_angle_global

        else:
            print('Did not detect a valid grasping branch')

            if self.save:
                img_rgb = self.crop(self.image).data
                save_img(img_rgb, pwd=pwd, name=self.name)
                img_rgb = self.image.data
                save_img(img_rgb, pwd=pwd, name=self.name + '_g')
            return False

        if self.save:
            open_dist_px = settings['open_dist_mm'] * self.px_per_mm
            finger_thickness_px = settings['finger_thinkness_mm'] * self.px_per_mm
            brightness = 0.85

            for frame_id in [self.LOCAL_FRAME_ID, self.ORIGINAL_FRAME_ID]:
                grasp_coord = self.grasp_point.get_coord(self.transform, frame_id)

                if frame_id == self.LOCAL_FRAME_ID:
                    grasp_angle = self.grasp_angle_local
                    img_rgb = self.crop(self.image).data.astype(np.uint8)

                elif frame_id == self.ORIGINAL_FRAME_ID:
                    grasp_angle = self.grasp_angle_global
                    img_rgb = np.array(self.image.data).astype(np.uint8)

                img_rgb_bright = change_brightness(img_rgb, brightness)
                branch_image = np.zeros(img_rgb_bright.shape[0:2], dtype=np.uint8)
                coords = np.rint(point2d.get_coord_list(points_keep, self.transform, frame_id)).astype(np.int)
                branch_image[coords[:, 1], coords[:, 0]] = 255

                if frame_id == self.ORIGINAL_FRAME_ID:
                    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                    branch_image = cv2.dilate(branch_image, kernel, iterations=1)

                visualize_skeleton(img_rgb_bright, branch_image, show_nodes=False, skeleton_color=(0, 0, 0),
                                   skeleton_width=4)
                plot_grasp_location(grasp_coord, grasp_angle, finger_width=minimum_grasp_length_px,
                                    finger_thickness=finger_thickness_px, finger_dist=open_dist_px, pwd=pwd,
                                    name=self.name + frame_id, linewidth=3)

        return success

    def crop(self, image):
        return image_crop(image, angle=-self.angle, bbox=self.bbox)

    def rescale(self, img):

        # TODO: this method is currently broken
        translation = translation_rot2or(self.shape, -self.angle)
        dist = np.sqrt(translation[0] ** 2 + translation[1] ** 2)
        x = self.bbox[0]
        y = self.bbox[0]

        if self.angle >= 0:
            transform = np.array(((-x + dist, -y),))
        if self.angle < 0:
            transform = np.array(((-x, -y + dist),))

        imgR = rotate(img, self.angle, resize=True)
        return add_border(imgR, transform, self.shape)

    def get_tomatoes(self, local=False):
        if local:
            # shape = self.bbox[:2]
            target_frame_id = self.LOCAL_FRAME_ID
        else:
            # shape = self.shape
            target_frame_id = self.ORIGINAL_FRAME_ID

        if self.centers is None:
            radii = []
            centers = [[]]
            com = []

        else:
            center_coords = point2d.get_coord_list(self.centers, self.transform, target_frame_id)
            com_coord = self.com.get_coord(self.transform, target_frame_id)
            radii = self.radii.tolist()

            # centers = np.around(centers_xy / scale).astype(int)
            # com = np.around(com_xy / scale).astype(int)
            row = [center[1] for center in center_coords]
            col = [center[0] for center in center_coords]
            centers = center_coords
            com = com_coord

        tomato = {'centers': centers, 'radii': radii, 'com': com, "row": row, "col": col}
        return tomato

    def get_peduncle_image(self, local=False):
        if local:
            return self.crop(self.peduncle).data
        else:
            return self.peduncle.data

    def get_main_peduncle_image(self, local=False):
        if local:
            return self.penduncle_main
        else:
            return self.rescale(self.penduncle_main)

    def get_peduncle(self, local=False):

        if local:
            frame_id = self.LOCAL_FRAME_ID
        else:
            frame_id = self.ORIGINAL_FRAME_ID

        junc_xy = point2d.get_coord_list(self.junction_points, self.transform, frame_id)
        end_xy = point2d.get_coord_list(self.end_points, self.transform, frame_id)
        peduncle = {'junctions': junc_xy, 'ends': end_xy}

        return peduncle

    def get_grasp_location(self, local=False):

        if local:
            frame_id = self.LOCAL_FRAME_ID
            angle = self.grasp_angle_local
        else:
            frame_id = self.ORIGINAL_FRAME_ID
            angle = self.grasp_angle_global
        if self.grasp_point is not None:
            xy = self.grasp_point.get_coord(self.transform, frame_id)
            grasp_pixel = np.around(xy).astype(int)
            row = grasp_pixel[1]
            col = grasp_pixel[0]
        else:
            row = None
            col = None

        grasp_location = {"row": row, "col": col, "angle": angle}
        return grasp_location

    def get_object_features(self):

        tomatoes = self.get_tomatoes()
        peduncle = self.get_peduncle()
        grasp_location = self.get_grasp_location()

        object_feature = {
            "grasp_location": grasp_location,
            "tomato": tomatoes,
            "peduncle": peduncle
        }
        return object_feature

    def get_tomato_visualization(self, local=False):
        if local is True:
            xy = point2d.get_coord_list(self.centers, self.transform, self.LOCAL_FRAME_ID)
            img = self.crop(self.image).data
        else:
            xy = point2d.get_coord_list(self.centers, self.transform, self.ORIGINAL_FRAME_ID)
            img = self.image.data
        fig = plt.figure()
        add_circles(img, xy, self.radii)
        return figure_to_image(fig)

    def get_rgb(self, local=False):
        if local:
            img = self.crop(self.image).data
        else:
            img = self.image.data
        return img

    def get_truss_visualization(self, local=False, save=False):
        pwd = os.path.join(self.pwd, '08_result')

        if local:
            frame_id = self.LOCAL_FRAME_ID
            grasp_angle = self.grasp_angle_local
            shape = self.shape  # self.bbox[2:4]
            zoom = True
        else:
            frame_id = self.ORIGINAL_FRAME_ID
            grasp_angle = self.grasp_angle_global
            shape = self.shape
            zoom = False

        xy_com = self.com.get_coord(self.transform, frame_id)
        xy_center = point2d.get_coord_list(self.centers, self.transform, frame_id)
        xy_grasp = self.grasp_point.get_coord(self.transform, frame_id)
        xy_junc = point2d.get_coord_list(self.junction_points, self.transform, frame_id)

        img = self.get_rgb(local=local)
        main_peduncle = self.penduncle_main

        fig = plt.figure()
        plot_image(img)

        branch_locs = []
        for branch_i, branch in enumerate(self.branch_data['junction-junction']):
            branch_coords = branch['coords']

            # filter coords near end and start node
            for branch_coord in branch_coords:
                branch_loc = branch_coord.get_coord(self.transform, frame_id)
                branch_locs.append(branch_loc)

        arr = np.zeros(shape, dtype=np.uint8)
        branch_locs = np.array(branch_locs)
        row_indices = np.around(branch_locs[:, 1]).astype(np.int)
        col_indices = np.around(branch_locs[:, 0]).astype(np.int)
        arr[row_indices, col_indices] = 1

        tomato = {'centers': xy_center, 'radii': self.radii, 'com': xy_com}
        plot_features(tomato=tomato, zoom=zoom)
        visualize_skeleton(img, arr, coord_junc=xy_junc, show_img=False)

        if (xy_grasp is not None) and (grasp_angle is not None):
            settings = self.settings['compute_grasp']
            if self.px_per_mm is not None:
                minimum_grasp_length_px = self.px_per_mm * settings['grasp_length_min_mm']
                open_dist_px = settings['open_dist_mm'] * self.px_per_mm
                finger_thickenss_px = settings['finger_thinkness_mm'] * self.px_per_mm
            else:
                minimum_grasp_length_px = settings['grasp_length_min_px']
            plot_grasp_location(xy_grasp, grasp_angle, finger_width=minimum_grasp_length_px,
                                finger_thickness=finger_thickenss_px, finger_dist=open_dist_px, linewidth=3)

        if save:
            save_fig(plt.gcf(), pwd, self.name)

        return figure_to_image(plt.gcf())

    def get_segments(self, local=False):
        if local:
            tomato = self.crop(self.tomato).data
            peduncle = self.crop(self.peduncle).data
            background = self.crop(self.background).data

        else:
            tomato = self.tomato.data
            peduncle = self.peduncle.data
            background = self.background.data

        return tomato, peduncle, background

    def get_segmented_image(self, local=False):
        tomato, peduncle, background = self.get_segments(local=local)
        image_RGB = self.get_rgb(local=local)
        data = stack_segments(image_RGB, background, tomato, peduncle)
        return data

    def get_color_components(self):
        return self.image_hue

    def save_results(self, name, local=False, pwd=None):
        if pwd is None:
            pwd = self.pwd

        tomato, peduncle, background = self.get_segments(local=local)
        img_rgb = self.get_rgb(local=local)
        plot_segments(img_rgb, background, tomato, peduncle, linewidth=0.5, pwd=pwd, name=name)

    @Timer("process image")
    def process_image(self):
        """
        Apply entire image processing pipeline
        returns: true if success, False if failed
        """

        self.color_space()

        success = self.segment_image()
        if success is False:
            return success

        success = self.filter_image()
        if success is False:
            return success

        success = self.rotate_cut_img()
        if success is False:
            return success

        success = self.detect_tomatoes()
        if success is False:
            return success

        success = self.detect_peduncle()
        if success is False:
            return success

        success = self.detect_grasp_location()
        return success

    def get_settings(self):
        return self.settings

    def set_settings(self, settings):
        """
        Overwirte the settings given only overwites the settings which are actually present in the given dict
        """

        for key_1 in settings:
            for key_2 in settings[key_1]:
                self.settings[key_1][key_2] = settings[key_1][key_2]


def main():
    i_start = 1
    i_end = 10
    N = i_end - i_start

    save = False
    drive = "backup"  # "UBUNTU 16_0"  #
    pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data",
                            "detect_truss")
    dataset = "lidl"  # "real_blue"  # "drawing" # "failures"  # 'test' # 'simpel_001'  # 'realistic_002'  #

    pwd_data = os.path.join(pwd_root, "data", dataset)
    pwd_results = os.path.join(pwd_root, "results", dataset)
    pwd_json = os.path.join(pwd_results, 'json')

    make_dirs(pwd_results)
    make_dirs(pwd_json)

    process_image = ProcessImage(use_truss=True,
                                 pwd=pwd_results,
                                 save=save)

    for count, i_tomato in enumerate(range(i_start, i_end)):
        print("Analyzing image ID %d (%d/%d)" % (i_tomato, count + 1, N))

        tomato_name = str(i_tomato).zfill(3)
        if i_tomato > 49:
            file_name = tomato_name + "_rgb" + ".png"
        else:
            file_name = tomato_name + ".png"

        rgb_data = load_rgb(pwd_data, file_name, horizontal=True)
        px_per_mm = load_px_per_mm(pwd_data, tomato_name)
        process_image.add_image(rgb_data, px_per_mm=px_per_mm, name=tomato_name)

        success = process_image.process_image()
        process_image.get_truss_visualization(local=True, save=True)

        json_data = process_image.get_object_features()

        pwd_json_file = os.path.join(pwd_json, tomato_name + '.json')
        with open(pwd_json_file, "w") as write_file:
            json.dump(json_data, write_file)

    if True:  # save is not True:
        plot_timer(Timer.timers['main'].copy(), threshold=0.02, pwd=pwd_results, name='main', title='Processing time',
                   startangle=-20)

    total_key = "process image"
    time_tot_mean = np.mean(Timer.timers[total_key]) / 1000
    time_tot_std = np.std(Timer.timers[total_key]) / 1000

    time_ms = Timer.timers[total_key]
    time_s = [x / 1000 for x in time_ms]

    time_min = min(time_s)
    time_max = max(time_s)

    print 'Processing time: {mean:.2f}s +- {std:.2f}s (n = {n:d})'.format(mean=time_tot_mean, std=time_tot_std, n=N)
    print 'Processing time lies between {time_min:.2f}s and {time_max:.2f}s (n = {n:d})'.format(time_min=time_min,
                                                                                                time_max=time_max, n=N)

    width = 0.5
    fig, ax = plt.subplots()

    ax.p1 = plt.bar(np.arange(i_start, i_end), time_s, width)

    plt.ylabel('time [s]')
    plt.xlabel('image ID')
    plt.title('Processing time per image')
    plt.rcParams["savefig.format"] = 'pdf'

    fig.show()
    fig.savefig(os.path.join(pwd_results, 'time_bar'), dpi=300)  # , bbox_inches='tight', pad_inches=0)


if __name__ == '__main__':
    main()
