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