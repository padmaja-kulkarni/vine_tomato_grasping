# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 16:05:23 2020

@author: taeke
"""

import numpy as np


def set_compute_grap_settings(grasp_length_min_px=30.0,
                              grasp_length_min_mm=15.0,
                              finger_thinkness_mm=10.0,
                              open_dist_mm=2*(0.037 - 0.0195)*1000):

    settings = {'grasp_length_min_px': grasp_length_min_px,
                'grasp_length_min_mm': grasp_length_min_mm,
                'finger_thinkness_mm': finger_thinkness_mm,
                'open_dist_mm': open_dist_mm}

    return settings
