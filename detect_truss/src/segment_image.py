# -*- coding: utf-8 -*-
"""
Created on Fri May 22 12:04:59 2020

@author: taeke
"""

## imports ##
import os  # os.sep
import cv2

# custom functions
from detect_truss.util import plot_segments
from detect_truss.util import load_rgb
from detect_truss.util import make_dirs

from detect_truss.segment_image import segment_truss
from detect_truss.ProcessImage import ProcessImage

# ls | cat -n | while read n f; do mv "$f" `printf "%03d.png" $n`; done
if __name__ == '__main__':

    N = 2  # tomato file to load
    extension = ".png"
    dataset ="failures" #  "depth_blue"  #
    save = True

    pwd_current = os.path.dirname(__file__)
    drive = "backup"  # "UBUNTU 16_0"  #
    pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data", "detect_truss")
    pwd_data = os.path.join(pwd_root, "data", dataset)
    pwd_results = os.path.join(pwd_root, "results", dataset)

    make_dirs(pwd_results)

    count = 0

    process_image = ProcessImage(use_truss=True,
                                 pwd=pwd_results,
                                 save=save)

    for i_tomato in range(1, N):
        tomato_ID = str(i_tomato).zfill(3)
        tomato_name = tomato_ID
        file_name = tomato_name + extension

        img_rgb = load_rgb(pwd_data, file_name, horizontal=True)
        process_image.add_image(img_rgb, name=tomato_name)
        process_image.color_space(compute_a=True)
        process_image.segment_image(radius=3.0)
        process_image.filter_image()
        count = count + 1
        print("completed image %d out of %d" % (count, N))
