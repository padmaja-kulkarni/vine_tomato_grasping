# -*- coding: utf-8 -*-
"""
Created on Fri Aug  7 12:16:30 2020

@author: taeke
"""

from matplotlib import pyplot as plt
from util import save_fig
import matplotlib as mpl
import cv2
import numpy as np
import os

plt.rcParams["savefig.bbox"] = 'tight' 
plt.rcParams['axes.titlesize'] = 20

drive = "backup"
pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data",
                        "flex_vision", "results")

fig = plt.figure(figsize=(0.1, 2.5))
dist_min = 0.5
dist_max = 0.7
n = 3
ticks = np.linspace(dist_min,dist_max,n)
cmap = mpl.cm.jet_r
norm = mpl.colors.Normalize(vmin=dist_min, vmax=dist_max)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])

cb1 = mpl.colorbar.ColorbarBase(plt.gca(), cmap=cmap, norm=norm, orientation='vertical')
cb1.set_label('depth [m]')
# plt.yticks([])
cb1.set_ticks(ticks, True)
# plt.yticks([0, 100], ['low', 'high'])
save_fig(plt.gcf(), pwd_root, 'color_bar_depth', no_ticks=False)


fig = plt.figure(figsize=(0.1, 3.5))

cmap = mpl.cm.hot_r
norm = mpl.colors.Normalize(vmin=0, vmax=100)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])

cb1 = mpl.colorbar.ColorbarBase(plt.gca(), cmap=cmap, norm=norm, orientation='vertical')
cb1.set_label('frequency', labelpad=-20)
# plt.yticks([])
cb1.set_ticks([0, 100], True)
plt.yticks([0, 100], ['low', 'high'])
save_fig(plt.gcf(), pwd_root, 'color_bar_hist', no_ticks=False)