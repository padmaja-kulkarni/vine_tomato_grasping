# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import numpy as np


def set_compute_grap_settings(grasp_length_min_px=30.0,
                              grasp_length_min_mm=20.0):
    settings = {'grasp_length_min_px': grasp_length_min_px,
                'grasp_length_min_mm': grasp_length_min_mm}

    return settings


def detect_grasp_location(branch_locs, branch_src_node_loc, branch_dst_node_loc, settings):
    minimum_grasp_length = settings['minimum_grasp_length']

    # filter coords near end and start node



    coords = []
    branches_i = []
    for branch_i, branch in enumerate(self.branch_data['junction-junction']):
        if branch['length'] > minimum_grasp_length:

            i_keep = []
            for ii, branch_loc in enumerate(branch_locs):
                src_node_dist = np.sqrt(np.sum(np.power(branch_loc - branch_src_node_loc, 2), 1))
                dst_node_dist = np.sqrt(np.sum(np.power(branch_loc - branch_dst_node_loc, 2), 1))
                if (dst_node_dist > 0.5*minimum_grasp_length) and (src_node_dist > 0.5*minimum_grasp_length):
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
        grasp_point = make_2d_point(self._LOCAL_FRAME_ID, xy=(loc[i, 0], loc[i, 1]))
        # graspR = graspL + [self.box[0], self.box[1]]
        # graspO = rot2or(graspR, self.DIM, np.deg2rad(-self.angle))

        self.grasp_point = grasp_point
        # self.graspR = graspR
        # self.graspO = graspO
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




def generate_image(locs, shape):
    branch_image = np.zeros(shape, dtype=np.uint8)
    for loc in locs:
        branch_image[loc[1], loc[0]] = 255