# -*- coding: utf-8 -*-
"""
Created on Fri May 22 12:04:59 2020

@author: taeke
"""

## imports ##
import os  # os.sep
import cv2

# custom functions
from flex_vision.utils.util import plot_segments
from flex_vision.utils.util import load_rgb
from flex_vision.utils.util import make_dirs

from flex_vision.detect_truss.segment_image import segment_truss
from flex_vision.detect_truss.ProcessImage import ProcessImage

# ls | cat -n | while read n f; do mv "$f" `printf "%03d.png" $n`; done
if __name__ == '__main__':

    i_start = 1  # tomato file to load
    i_end = 2
    N = i_end - i_start

    extension = ".png"
    dataset = "lidl"  # "failures" #
    save = True

    pwd_current = os.path.dirname(__file__)
    drive = "backup"  # "UBUNTU 16_0"  #
    pwd_root = os.path.join(os.sep, "media", "taeke", drive, "thesis_data", "detect_truss")
    pwd_data = os.path.join(pwd_root, "data", dataset)
    pwd_results = os.path.join(pwd_root, "results", dataset)

    make_dirs(pwd_results)
    process_image = ProcessImage(use_truss=True,
                                 pwd=pwd_results,
                                 save=save)
    radii = [None] #  [1.5]  # 0.5, 1.0, 1.5, 2.0, 3.0

    for radius in radii:
        for count, i_tomato in enumerate(range(i_start, i_end)):
            print("Analyzing image ID %d (%d/%d)" % (i_tomato, count + 1, N))

            tomato_ID = str(i_tomato).zfill(3)
            tomato_name = tomato_ID
            file_name = tomato_name + extension

            img_rgb = load_rgb(file_name, pwd_data, horizontal=True)
            process_image.add_image(img_rgb, name=tomato_name)
            process_image.color_space(compute_a=True)
            process_image.segment_image(radius=radius)
            process_image.filter_image(folder_name=str(radius))
            count = count + 1
            print("completed image %d out of %d" % (count, N))
