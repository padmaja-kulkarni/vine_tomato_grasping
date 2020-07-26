# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 10:31:12 2020

@author: taeke
"""

    def detect_tomatoes_global(self):
        # TODO: crop image into multiple 'tomatoes', and detect tomatoes in each crop
        tomatoFilteredLBlurred = cv2.GaussianBlur(self.tomato, self.blur_size, 0)
        minR = self.W/20 # 6
        maxR = self.W/8
        minDist = self.W/10

        circles = cv2.HoughCircles(tomatoFilteredLBlurred, cv2.HOUGH_GRADIENT, 5, minDist,
                                   param1=self.param1,param2=self.param2, minRadius=minR, maxRadius=maxR)

        if circles is None:
            warnings.warn("Failed to detect any circle!")
            centersO = None
            radii = None
        else:
            centersO = np.matrix(circles[0][:,0:2])
            radii = circles[0][:,2]

        self.centersO = centersO
        self.radii = radii