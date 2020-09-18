# -*- coding: utf-8 -*-


## imports ##
import os  # os.sep
import warnings

import numpy as np
import tf2_ros
import cv2
import tf2_geometry_msgs
import json

from skimage.transform import rotate
import rospy

from image import Image, compute_angle, add, compute_bbox, image_rotate, image_crop, image_cut

from timer import Timer

# custom functions
from util import add_border

from util import translation_rot2or
from util import add_circles, plot_features

from util import make_dirs
from util import save_img, figure_to_image
from util import load_rgb
from util import stack_segments
from util import plot_timer, plot_grasp_location
from util import change_brightness, plot_segments

from point_2d import make_2d_transform, make_2d_point, make_2d_points

from matplotlib import pyplot as plt

from filter_segments import filter_segments
from detect_peduncle_2 import detect_peduncle, visualize_skeleton, set_detect_peduncle_settings
from detect_tomato import detect_tomato, set_detect_tomato_settings
from compute_grasp import set_compute_grap_settings
from segment_image import segment_truss

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
    _ORIGINAL_FRAME_ID = 'original'
    _LOCAL_FRAME_ID = 'local'

    name_space = 'main'

    dtype = np.uint8

    def __init__(self,
                 use_truss=True,
                 save=False,
                 pwd='',
                 name='tomato',
                 ext='png'
                 ):

        self.ext = ext
        self.save = save
        self.use_truss = use_truss
        self.pwd = pwd
        self.name = name
        # self.use_ransac = False

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

        # image
        settings = {}
        settings['detect_tomato'] = set_detect_tomato_settings()
        settings['detect_peduncle'] = set_detect_peduncle_settings()
        settings['grasp_location'] = set_compute_grap_settings()
        self.settings = settings

        # detect junctions
        self.distance_threshold = 10

        # init buffer
        self.buffer_core = tf2_ros.BufferCore(rospy.Time(10))

    def add_image(self, data, px_per_mm=None, name=None):
        image = Image(data)
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

        # self.image_hue = imHSV[:, :, 0]
        if self.save:
            save_img(self.image_hue, pwd, self.name + '_h_raw') # color_map='hsv'
            save_img(self.image_a, pwd, self.name + '_a_raw')

            save_img(self.image_hue, pwd, self.name + '_h', color_map='HSV') # color_map='hsv'
            save_img(self.image_a, pwd, self.name + '_a', color_map='Lab')

    @Timer("segmentation", name_space)
    def segment_image(self, radius=None):
        if radius is None:
            pwd = os.path.join(self.pwd, '02_segment')
            radius = 1.5
        else:
            pwd = os.path.join(self.pwd, '02_segment', str(radius))

        success = True
        background, tomato, peduncle = segment_truss(self.image_hue,
                                                     img_a=self.image_a,
                                                     radius=radius,
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
        pwd = os.path.join(self.pwd, '03_filter')
        if folder_name is not None:
            pwd = os.path.join(pwd, folder_name)


        tomato_f, peduncle_f, background_f = filter_segments(self.tomato.data,
                                                             self.peduncle.data,
                                                             self.background.data)

        self.tomato.data = tomato_f
        self.peduncle.data = peduncle_f
        self.background.data = background_f

        if self.save:
            self.save_results(self.name, pwd=pwd)

        if self.tomato.is_empty():
            warnings.warn("filter segments: no pixel has been classified as tomato")
            return False

        return True

    @Timer("cropping", name_space)
    def rotate_cut_img(self):
        pwd = os.path.join(self.pwd, '04_crop')

        if self.peduncle.is_empty():
            warnings.warn("Cannot rotate based on peduncle, since it does not exist!")
            angle = 0
        else:
            angle = compute_angle(self.peduncle.data)  # [rad]

        tomato_rotate = image_rotate(self.tomato, -angle)
        peduncle_rotate = image_rotate(self.peduncle, -angle)
        truss_rotate = add(tomato_rotate, peduncle_rotate)

        if truss_rotate.is_empty():
            warnings.warn("Cannot crop based on tomato, since it does not exist!")
            return False

        bbox = compute_bbox(truss_rotate.data)
        x = bbox[0]
        y = bbox[1]

        # get origin
        translation = translation_rot2or(self.shape, -angle)
        dist = np.sqrt(translation[0] ** 2 + translation[1] ** 2)

        if angle >= 0:
            xy = (-x + dist, -y)
        if angle < 0:
            xy = (-x, -y + dist)

        # make and add transform to the buffer
        transform = make_2d_transform(self._ORIGINAL_FRAME_ID,
                                      self._LOCAL_FRAME_ID,
                                      xy=xy,
                                      angle=angle)

        self.buffer_core.set_transform(transform, "default_authority")
        self.bbox = bbox
        self.angle = angle

        self.tomato_crop = self.cut(tomato_rotate)
        self.peduncle_crop = self.cut(peduncle_rotate)
        self.image_crop = self.crop(self.image)
        self.truss_crop = self.cut(truss_rotate)

        if self.save:
            self.save_results(self.name, pwd=pwd, local=True)

    @Timer("tomato detection", name_space)
    def detect_tomatoes(self):
        pwd = os.path.join(self.pwd, '05_tomatoes')

        if self.peduncle.is_empty():
            return False

        if self.save:
            img_bg = self.image_crop.data  # self.get_segmented_image(local = True)
        else:
            img_bg = self.image_crop.data

        centers, radii, com = detect_tomato(self.truss_crop.data,
                                            self.settings['detect_tomato'],
                                            px_per_mm=self.px_per_mm,
                                            img_rgb=img_bg,
                                            save=self.save,
                                            pwd=pwd,
                                            name=self.name)

        # convert to 2D points        
        center_points = []
        if centers is not None:
            for center in centers:
                center_points.append(make_2d_point(self._LOCAL_FRAME_ID, (center[0, 0], center[0, 1])))

        if com is not None:
            success = True
            com_point = make_2d_point(self._LOCAL_FRAME_ID, (com[0, 0], com[0, 1]))
        else:
            com_point = None
            success = False

        self.com = com_point
        self.centers = center_points
        self.radii = radii
        return success

    @Timer("peduncle detection", name_space)
    def detect_peduncle(self):
        pwd = os.path.join(self.pwd, '06_peduncle')
        success = True

        if self.save:
            img_bg = change_brightness(self.get_segmented_image(local=True), 0.85)  # self.image_crop.data #
        else:
            img_bg = self.image_crop.data
        # bg_img = self.image_crop.data, 0.85)

        mask, branch_data, junc_coords, end_coords = detect_peduncle(self.peduncle_crop.data,
                                            self.settings['detect_peduncle'],
                                            px_per_mm=self.px_per_mm,
                                            save=self.save,
                                            bg_img=img_bg,
                                            name=self.name,
                                            pwd=pwd)

        # convert to 2D points
        junction_points = []
        for junction in junc_coords:
            junction_points.append(make_2d_point(self._LOCAL_FRAME_ID, junction))

        end_points = []
        for end in end_coords:
            end_points.append(make_2d_point(self._LOCAL_FRAME_ID, end))

        for branch_type in branch_data:
            for i, branch in enumerate(branch_data[branch_type]):
                for lbl in ['coords', 'src_node_coord', 'dst_node_coord', 'center_node_coord']:
                    coord = branch[lbl]
                    if lbl == 'coords':
                        branch_data[branch_type][i][lbl] = make_2d_points(self._LOCAL_FRAME_ID, coord)
                    else:
                        branch_data[branch_type][i][lbl] = make_2d_point(self._LOCAL_FRAME_ID, coord)

        self.junction_points = junction_points
        self.end_points = end_points
        self.penduncle_main = mask
        self.branch_data = branch_data
        return success

    @Timer("detect grasp location", name_space)
    def detect_grasp_location(self):
        pwd = os.path.join(self.pwd, '07_grasp')
        success = True

        com = self.get_xy(self.com, self._LOCAL_FRAME_ID)
        settings = self.settings['grasp_location']

        # set dimensions
        if self.px_per_mm is not None:
            minimum_grasp_length_px = self.px_per_mm * settings['grasp_length_min_mm']
        else:
            minimum_grasp_length_px = settings['grasp_length_min_px']

        coords = []
        branches_i = []
        for branch_i, branch in enumerate(self.branch_data['junction-junction']):
            if branch['length'] > minimum_grasp_length_px:
                branch_coords = branch['coords']

                # filter coords near end and start node
                branch_locs = self.get_xy(branch_coords, self._LOCAL_FRAME_ID)
                branch_src_node_loc = self.get_xy(branch['src_node_coord'], self._LOCAL_FRAME_ID)
                branch_dst_node_loc = self.get_xy(branch['dst_node_coord'], self._LOCAL_FRAME_ID)

                i_keep = []
                for ii, branch_loc in enumerate(branch_locs):
                    src_node_dist = np.sqrt(np.sum(np.power(branch_loc - branch_src_node_loc, 2), 1))
                    dst_node_dist = np.sqrt(np.sum(np.power(branch_loc - branch_dst_node_loc, 2), 1))
                    if (dst_node_dist > 0.5 * minimum_grasp_length_px) and (
                            src_node_dist > 0.5 * minimum_grasp_length_px):
                        i_keep.append(ii)

                branch_coords_keep = [branch_coords[i] for i in i_keep]
                coords.extend(branch_coords_keep)
                branches_i.extend([branch_i] * len(branch_coords_keep))

        if len(branches_i) > 0:
            loc = self.get_xy(coords, self._LOCAL_FRAME_ID)
            dist = np.sqrt(np.sum(np.power(loc - com, 2), 1))
            i = np.argmin(dist)
            branch_i = branches_i[i]
            grasp_angle_local = self.branch_data['junction-junction'][branch_i]['angle'] / 180.0 * np.pi

            grasp_angle_global = -self.angle + grasp_angle_local
            grasp_point = make_2d_point(self._LOCAL_FRAME_ID, xy=loc[i, :])  # , loc[i, 1])

            self.grasp_point = grasp_point
            self.grasp_angle_local = grasp_angle_local
            self.grasp_angle_global = grasp_angle_global

        else:
            print('Did not detect a valid grasping branch')

            if self.save:
                img_rgb = self.crop(self.image).data
                save_img(img_rgb, pwd=pwd, name=self.name, ext=self.ext)
                img_rgb = self.image.data
                save_img(img_rgb, pwd=pwd, name=self.name + '_g', ext=self.ext)
            return False

        if self.save:
            xy_local = self.get_xy(grasp_point, self._LOCAL_FRAME_ID)
            img_rgb = self.crop(self.image).data.astype(np.uint8)

            branch_image = np.zeros(img_rgb.shape[0:2], dtype=np.uint8)
            locs = np.rint(self.get_xy(coords, self._LOCAL_FRAME_ID)).astype(np.int)  # , dtype=int
            branch_image[locs[:, 1], locs[:, 0]] = 255

            visualize_skeleton(img_rgb, branch_image, show_nodes=False, skeleton_color=(0, 0, 0), skeleton_width=4)
            plot_grasp_location(xy_local, grasp_angle_local, l=minimum_grasp_length_px, pwd=pwd, name=self.name,
                                linewidth=2)

            xy_global = self.get_xy(grasp_point, self._ORIGINAL_FRAME_ID)
            img_rgb = np.array(self.image.data).astype(np.uint8)

            branch_image = np.zeros(img_rgb.shape[0:2], dtype=np.uint8)
            locs = np.rint(self.get_xy(coords, self._ORIGINAL_FRAME_ID)).astype(np.int)
            branch_image[locs[:, 1], locs[:, 0]] = 255

            # branch_image = Image(branch_image)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            # branch_image.open_close(kernel)
            # cv2.morphologyEx(branch_image, cv2.MORPH_CLOSE, kernel)

            branch_image = cv2.dilate(branch_image, kernel, iterations=1)

            visualize_skeleton(img_rgb, branch_image, skeletonize=True, show_nodes=False, skeleton_color=(0, 0, 0))
            plot_grasp_location(xy_global, grasp_angle_global, l=minimum_grasp_length_px, pwd=pwd,
                                name=self.name + '_g')

        return success

    def transform_points(self, points, targer_frame_id):

        if isinstance(points, list):
            points_new = []
            for point in points:
                point_transform = self.buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id,
                                                                         rospy.Time(0))
                points_new.append(tf2_geometry_msgs.do_transform_pose(point, point_transform))

        else:
            point = points
            point_transform = self.buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id,
                                                                     rospy.Time(0))
            points_new = tf2_geometry_msgs.do_transform_pose(point, point_transform)

        return points_new

    def get_xy(self, points, target_frame_id):
        if points is None:
            return None
        points_new = self.transform_points(points, target_frame_id)

        if isinstance(points, list):
            xy = []
            for point in points_new:
                xy.append((point.pose.position.x, point.pose.position.y))
        else:
            xy = (points_new.pose.position.x, points_new.pose.position.y)

        return np.array(xy, ndmin=2)

    def crop(self, image):
        return image_crop(image, angle=-self.angle, bbox=self.bbox)

    def rotate(self, image):
        return image_rotate(image, angle=-self.angle)

    def cut(self, image):
        return image_cut(image, bbox=self.bbox)

    def rescale(self, img):

        # TODO: fix
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
            target_frame_id = self._LOCAL_FRAME_ID
            scale = 1.0
        else:
            # shape = self.shape
            target_frame_id = self._ORIGINAL_FRAME_ID
            scale = self.scale

        # mask_empty = np.zeros(shape, self.dtype)

        if self.centers is None:
            #            row = []
            #            col = []
            radii = []
            # mask = mask_empty
            centers = [[]]
            com = []

        else:
            centers_xy = self.get_xy(self.centers, target_frame_id)
            com_xy = self.get_xy(self.com, target_frame_id)
            radii = (self.radii / scale).astype(int).tolist()

            centers = np.around(centers_xy / scale).astype(int)
            com = np.around(com_xy / scale).astype(int)
            row = centers[:, 1].tolist()
            col = centers[:, 0].tolist()
            centers = centers.tolist()
            com = com.tolist()
        #            centers = []
        #            for row, col in zin(tomatoRow, tomatoCol):
        #                center = [row, col]
        #                centers.append(center)

        #             mask = add_circles(mask_empty, centers, radii, color = (255), thickness = -1).tolist()

        tomato = {'centers': centers, 'radii': radii, 'com': com, "row": row, "col": col}  # ,  "mask": mask,
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
        #        peduncle_mask = self.get_peduncle_image(local = local)
        #        penduncle_main = self.get_main_peduncle_image(local = local)
        #        peduncle = {"mask": peduncle_mask,
        #                    "mask_main": penduncle_main}

        if local:
            frame_id = self._LOCAL_FRAME_ID
        else:
            frame_id = self._ORIGINAL_FRAME_ID

        junc_xy = self.get_xy(self.junction_points, frame_id).tolist()
        end_xy = self.get_xy(self.end_points, frame_id).tolist()
        # junc_xy, end_xy = self.get_node_xy(local=local)
        peduncle = {'junctions': junc_xy, 'ends': end_xy}

        return peduncle

    def get_grasp_location(self, local=False):

        if local:
            frame_id = self._LOCAL_FRAME_ID
            angle = self.grasp_angle_local
            scale = 1
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            angle = self.grasp_angle_global
            scale = self.scale

        if self.grasp_point is not None:
            xy = self.get_xy(self.grasp_point, frame_id)
            graspPixel = np.around(xy / scale).astype(int)
            row = graspPixel[0, 1]
            col = graspPixel[0, 0]
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
        if local == True:
            xy = self.get_xy(self.centers, self._LOCAL_FRAME_ID)
            img = self.crop(self.image).data
        elif local == False:
            xy = self.get_xy(self.centers, self._ORIGINAL_FRAME_ID)
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

    def get_truss_visualization(self, local=False):
        if local:
            frame_id = self._LOCAL_FRAME_ID
            grasp_angle = self.grasp_angle_local
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            grasp_angle = self.grasp_angle_global

        xy_com = self.get_xy(self.com, frame_id)
        xy_center = self.get_xy(self.centers, frame_id)
        xy_grasp = self.get_xy(self.grasp_point, frame_id)
        xy_junc = self.get_xy(self.junction_points, frame_id).tolist()
        xy_end = self.get_xy(self.end_points, frame_id).tolist()

        img = self.get_rgb(local=local)
        tomato, peduncle, background = self.get_segments(local=local)
        main_peduncle = self.penduncle_main
        # img_peduncle = self.get_peduncle_image(local = local)

        plot_segments(img, background, tomato, peduncle)
        # add_circles(img, xy_center, radii=self.radii, color=(0, 0, 0), thickness=1)
        tomato = {'centers': xy_center, 'radii': self.radii, 'com': xy_com}
        plot_features(tomato=tomato)
        visualize_skeleton(img, main_peduncle, coord_junc=xy_junc, coord_end=xy_end, show_img=False)

        if (xy_grasp is not None) and (grasp_angle is not None):
            settings = self.settings['grasp_location']
            if self.px_per_mm is not None:
                minimum_grasp_length_px = self.px_per_mm * settings['grasp_length_min_mm']
            else:
                minimum_grasp_length_px = settings['grasp_length_min_px']
            plot_grasp_location(xy_grasp, grasp_angle, l=minimum_grasp_length_px)

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
        plot_segments(img_rgb, background, tomato, peduncle, linewidth=1.1, pwd=pwd, name=name)

    @Timer("process image")
    def process_image(self):

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
        ''' Overwirte the settings given '''
        #        self.settings = settings
        for key_1 in settings:
            for key_2 in settings[key_1]:
                self.settings[key_1][key_2] = settings[key_1][key_2]


if __name__ == '__main__':
    i_start = 1
    i_end = 50
    N = i_end - i_start

    save = True
    drive = "backup" # "UBUNTU 16_0"  #
    pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data",
                            "detect_truss")
    dataset = "depth_blue"  # "real_blue"  # "drawing" #

    pwd_data = os.path.join(pwd_root, "data", dataset)
    pwd_results = os.path.join(pwd_root, "results", dataset)
    pwd_json = os.path.join(pwd_results, 'json')

    make_dirs(pwd_results)
    make_dirs(pwd_json)

    process_image = ProcessImage(use_truss=True,
                                 pwd=pwd_results,
                                 save=save)

    # process_image.use_ransac = False

    for count, i_tomato in enumerate(range(i_start, i_end)):
        print("Analyzing image ID %d (%d/%d)" % (i_tomato, count + 1, N))

        tomato_name = str(i_tomato).zfill(3)
        file_name = tomato_name + ".png"

        rgb_data = load_rgb(pwd_data, file_name, horizontal=True)
        px_per_mm = load_px_per_mm(pwd_data, tomato_name)
        process_image.add_image(rgb_data, px_per_mm=px_per_mm, name=tomato_name)

        success = process_image.process_image()

        json_data = process_image.get_object_features()

        pwd_json_file = os.path.join(pwd_json, tomato_name + '.json')
        with open(pwd_json_file, "w") as write_file:
            json.dump(json_data, write_file)

    if save is not True:
        plot_timer(Timer.timers['main'].copy(), threshold=0.02, pwd=pwd_results, name='main', title='Processing time')
        # plot_timer(Timer.timers['peduncle'].copy(), N=N, threshold=0.02, pwd=pwd_results, name='peduncle',
        #            title='Processing time peduncle')

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
    # plt.rc('ytick', labelsize=16)

    fig.show()
    fig.savefig(os.path.join(pwd_results, 'time_bar'), dpi = 300) #, bbox_inches='tight', pad_inches=0)
