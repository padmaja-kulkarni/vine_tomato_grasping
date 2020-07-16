# -*- coding: utf-8 -*-
"""
Created on Thu Jul 16 22:37:15 2020

@author: taeke
"""

# find CoM
# comR = comL + self.box[0:2]
# comO = rot2or(comR, self.DIM, -np.deg2rad(self.angle))

# centersR = centers + self._bbox[0:2]
# centersO = rot2or(centersR, self._image_RGB.get_dimensions(), -self._angle) # np.deg2rad(

xy_local = self.get_xy(center_points, self._LOCAL_FRAME_ID)
plot_circles(self.crop(self._image_RGB).get_data(), xy_local, radii, pwd = self.pwdProcess, name = '05_a')

#xy_rotated = self.get_xy(center_points, self._ROTATED_FRAME_ID)
#plot_circles(self.rotate(self._image_RGB).get_data(), xy_rotated, radii, pwd = self.pwdProcess, name = '05_a')

xy_original = self.get_xy(center_points, self._ORIGINAL_FRAME_ID)
plot_circles(self._image_RGB.get_data(), xy_original, radii, pwd = self.pwdProcess, name = '05_a')