# -*- coding: utf-8 -*-


## imports ##
import os # os.sep
import warnings


import numpy as np
import tf2_ros
import cv2
import tf2_geometry_msgs

from skimage.transform import rotate
from skimage.morphology import skeletonize

import rospy
from flex_grasp.msg import ImageProcessingSettings

from image import Image, compute_angle, add, compute_bbox, image_rotate, image_crop

# custom functions
from util import add_border
from util import remove_all_blobs
from util import segmentation_truss_real, segmentation_tomato_real
from util import segmentation_truss_sim
from util import translation_rot2or
from util import add_circles, add_contour


from util import save_img
from util import load_rgb
from util import stack_segments
from util import plot_circles
from util import change_brightness

from point_2d import make_2d_transform, make_2d_point

from detect_peduncle import detect_peduncle
from detect_tomato import detect_tomato

class ProcessImage(object):

    def __init__(self, 
                 use_truss = True,
                 camera_sim = True, 
                 tomatoName = 'tomato', 
                 saveIntermediate = False, 
                 pwdProcess = '', 
                 saveFormat = 'png'               
                 ):

        self.saveFormat = saveFormat

        
        self.saveIntermediate = saveIntermediate

        self.camera_sim = camera_sim
        self.use_truss = use_truss
        self.imMax = 255
        self.pwdProcess = pwdProcess
        self.tomatoName = tomatoName

        # image
        self.width_desired = 1280

        # filtering
        self.filter_tomato_diameter = 11
        self.filter_tomato_shape = cv2.MORPH_ELLIPSE
        self.filter_penduncle_diameter = 5
        self.filter_penduncle_shape = cv2.MORPH_ELLIPSE

        # detect tomatoes
        self.blur_size = (3,3)
        self.tomato_radius_min = 8
        self.tomato_radius_max = 4
        self.tomato_distance_min = self.tomato_radius_max
        self.dp = 4
        self.param1 = 20
        self.param2 = 50
        self.ratio_threshold = 0.5        
        
        self.peduncle_element = (20, 2)
        
        # detect junctions
        self.distance_threshold = 10

        # init buffer
        self._buffer_core = tf2_ros.BufferCore(rospy.Time(10))
        
        # frame ids
        self._ORIGINAL_FRAME_ID = 'original'
        self._ROTATED_FRAME_ID = 'rotated'
        self._LOCAL_FRAME_ID = 'local'

    def add_image(self, data):
        
        
        image = Image(data)
        image.rescale(self.width_desired)        
        self._image_RGB = image        
        
        if self.saveIntermediate:
            save_img(self._image_RGB._data, self.pwdProcess, '01', saveFormat = self.saveFormat)

    def color_space(self):
        
        imHSV = cv2.cvtColor(self._image_RGB._data, cv2.COLOR_RGB2HSV)
        imLAB = cv2.cvtColor(self._image_RGB._data, cv2.COLOR_RGB2LAB)
        self._image_hue = imHSV[:, :, 0]
        self._image_saturation = imHSV[:, :, 1]
        self._image_A  = imLAB[:, :, 1]
        
    def segment_truss(self):

        success = True
        if self.camera_sim:
            background, tomato, peduncle = segmentation_truss_sim(self._image_saturation, self._image_hue, self._image_A, self.imMax)

        else:
            if self.use_truss:
                background, tomato, peduncle = segmentation_truss_real(self._image_hue, self.imMax)
            else:
                background, tomato, peduncle = segmentation_tomato_real(self._image_A, self.imMax)
        
        self._background = Image(background)
        self._tomato = Image(tomato)
        self._peduncle = Image(peduncle)
        

        if np.all((tomato == 0)):
            warnings.warn("Segment truss: no pixel has been classified as tomato")
            success = False

        if self.saveIntermediate:
            self.save_results('02')

        return success

    def filter_img(self):
#        tomato_kernel = cv2.getStructuringElement(self.filter_tomato_shape, (self.filter_tomato_diameter, self.filter_tomato_diameter))
#        self._tomato.open_close(tomato_kernel)
#
#        peduncle_kernel = cv2.getStructuringElement(self.filter_penduncle_shape, (self.filter_penduncle_diameter, self.filter_penduncle_diameter))
#        self._peduncle.close_open(peduncle_kernel)
#        self._peduncle.remove_blobs()
#
#        self._background = Image(cv2.bitwise_not(self._tomato.get_data()))
    
        self._tomato._data, self._peduncle._data, self._background._data = remove_all_blobs(self._tomato.get_data(),
                         self._peduncle.get_data(), self._background.get_data(), self.imMax)

        if self.saveIntermediate:
            self.save_results('03')
            
        if self._tomato.is_empty():
            warnings.warn("filter segments: no pixel has been classified as tomato")
            return False
            
        return True

    def rotate_cut_img(self):
    
        if self._peduncle.is_empty():
            warnings.warn("Cannot rotate based on peduncle, since it does not exist!")
            angle = 0
        else:
            angle = compute_angle(self._peduncle.get_data()) # [rad] 

        tomato_rotate = image_rotate(self._tomato, -angle)
        peduncle_rotate = image_rotate(self._peduncle, -angle)        
        truss = add(tomato_rotate, peduncle_rotate)
        
        if truss.is_empty():
            warnings.warn("Cannot crop based on tomato, since it does not exist!")
            return False

        bbox = compute_bbox(truss.get_data())
        x = bbox[0]
        y = bbox[1]
        w = bbox[2]
        h = bbox[3]

        #get origin
        translation = translation_rot2or(self._image_RGB.get_dimensions(), -angle)
        dist = np.sqrt(translation[0]**2 + translation[1]**2)
        # transform = make_2d_transform(self._ROTATED_FRAME_ID, self._ORIGINAL_FRAME_ID,  xy = (-150, -220) , angle = -angle)
        # self._buffer_core.set_transform(transform, "default_authority")   
        if angle >= 0:
            transform = make_2d_transform(self._ORIGINAL_FRAME_ID,  self._LOCAL_FRAME_ID, xy = (-x + dist,-y), angle = angle)
        if angle < 0:    
            transform = make_2d_transform(self._ORIGINAL_FRAME_ID,  self._LOCAL_FRAME_ID, xy = (-x,-y + dist), angle = angle)
        
        self._buffer_core.set_transform(transform, "default_authority")   

        self._w = w
        self._h = h
        self._bbox = bbox
        self._angle = angle
        
        if self.saveIntermediate:
            self.save_results('04', crop = True)

    def detect_tomatoes(self):
        success = True
        
        tomato_crop = self.crop(self._tomato)
        peduncle_crop = self.crop(self._peduncle)
        truss_crop = add(tomato_crop, peduncle_crop).get_data()
    
        bg_image = self.crop(self._image_RGB).get_data()
        bg_img = change_brightness(bg_image, 0.85)
    
        centers, radii, com = detect_tomato(truss_crop, imageRGB=bg_img, 
                  blur_size=self.blur_size, radius_min=self.tomato_radius_min, 
                  radius_max=self.tomato_radius_max, 
                  distance_min=self.tomato_distance_min,
                  dp = self.dp, param1 = self.param1, param2 = self.param2,
                  ratio_threshold = self.ratio_threshold,
                  save = self.saveIntermediate, pwd = self.pwdProcess, name = self.tomatoName)
            
        # convert to 2D points
        center_points = []
        for center in centers:
            center_points.append(make_2d_point(self._LOCAL_FRAME_ID, (center[0,0], center[0,1])))
                
        if com is not None:
            success is True
            com_point =  make_2d_point(self._LOCAL_FRAME_ID, (com[0,0], com[0,1]))
        else:
            com_point = None
            success = False

        self.com = com_point
        self.centers = center_points
        self.radii = radii             
        return success

    def detect_peduncle(self):
        distance_threshold = 10
        success = True    
    
        bg_img = self.crop(self._image_RGB).get_data()
        bg_img = change_brightness(bg_img, 0.85)
        peduncleL = self.crop(self._peduncle).get_data()
        
        penduncle_main, branch_center = detect_peduncle(peduncleL, 
                                                        distance_threshold, 
                                                        save = self.saveIntermediate, 
                                                        bg_img = bg_img, 
                                                        name = '06', 
                                                        pwd = self.pwdProcess)
        
        self.penduncle_main = penduncle_main
        self.junc_branch_center = branch_center
        return success

    def detect_grasp_location(self, strategy = 'cage'):
        success = True        

        if strategy== "cage":
            com = self.get_xy(self.com, self._LOCAL_FRAME_ID)
 
            if self.junc_branch_center.size > 0: # self.junc_branch_center:        
                print('Detected a junction')
                loc = self.junc_branch_center
                dist = np.sqrt(np.sum(np.power(loc - com, 2), 1))
                
            else:
                print('Did not detect a junction')                
                skeleton = skeletonize(self.penduncle_main/self.imMax)
                col, row = np.nonzero(skeleton)
                loc = np.transpose(np.matrix(np.vstack((row, col))))
                
                dist = np.sqrt(np.sum(np.power(loc - com, 2), 1))
                
            i = np.argmin(dist)
            grasp_angle = self._angle
            
        elif strategy== "pinch":
            
            skeleton = skeletonize(self.peduncleL/self.imMax)
            col, row = np.nonzero(skeleton)
            loc = np.transpose(np.matrix(np.vstack((row, col))))
            
            dist0 = np.sqrt(np.sum(np.power(loc - self.centersL[0,:], 2), 1))
            dist1 = np.sqrt(np.sum(np.power(loc - self.centersL[1,:], 2), 1))
            dist = np.minimum(dist0, dist1)
            i = np.argmax(dist)
            
            grasp_angle = self._angle
        else:
            print("Unknown grasping strategy")
            return False
        

        grasp_point = make_2d_point(self._LOCAL_FRAME_ID, xy = (loc[i, 0], loc[i, 1]))
        # graspR = graspL + [self.box[0], self.box[1]]
        # graspO = rot2or(graspR, self.DIM, np.deg2rad(-self.angle))

        self.grasp_point = grasp_point
        # self.graspR = graspR
        # self.graspO = graspO
        self.grasp_angle = grasp_angle

        if self.saveIntermediate: # True: # self.saveIntermediate:
            xy_local = self.get_xy(grasp_point, self._LOCAL_FRAME_ID)
            plot_circles(self.crop(self._image_RGB).get_data(), xy_local, 10, pwd = self.pwdProcess, name = '06')
            
            
#            xy_local = self.get_xy(grasp_point, self._ORIGINAL_FRAME_ID)
#            plot_circles(self._image_RGB.get_data(), xy_local, 10, pwd = self.pwdProcess, name = '06')
            
        return success
        
    def transform_points(self, points, targer_frame_id):
        
        if isinstance(points, list):
            points_new = []
            for point in points:
                point_transform = self._buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id, rospy.Time(0)) 
                points_new.append(tf2_geometry_msgs.do_transform_pose(point, point_transform))
                
        else:
            point = points
            point_transform = self._buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id, rospy.Time(0)) 
            points_new = tf2_geometry_msgs.do_transform_pose(point, point_transform)          
        
        return points_new
        
    def get_xy(self, points, target_frame_id):
        
        points_new = self.transform_points(points, target_frame_id)        
        
        if isinstance(points, list):
            xy = []
            for point in points_new:
                xy.append((point.pose.position.x, point.pose.position.y))
        else:
            xy = (points_new.pose.position.x, points_new.pose.position.y)
            
        return np.array(xy, ndmin=2)

    def crop(self, image):
        return image_crop(image, angle = -self._angle, bbox = self._bbox)
        
    def rotate(self, image):
        return image_rotate(image, angle = -self._angle)

    def rescale(self, img):
        
        # TODO: fix
        translation = translation_rot2or(self._image_RGB.get_dimensions(), -self._angle)
        dist = np.sqrt(translation[0]**2 + translation[1]**2)
        x = self._bbox[0]
        y = self._bbox[0]
  
        if self._angle >= 0:
            transform = np.array(((-x + dist,-y),))
        if self._angle < 0:    
            transform = np.array(((-x,-y + dist),))
        
        imgR = np.uint8(self.imMax*rotate(img, self._angle, resize=True))
        return add_border(imgR, transform, self._image_RGB.get_dimensions())

    def get_tomatoes(self, local = False):
        if local:
            mask_empty = np.zeros(self._image_RGB.get_dimensions(), np.uint8)      
            target_frame_id = self._LOCAL_FRAME_ID
            scale = 1
        else:
            mask_empty = np.zeros((self._h, self._w), np.uint8)   
            target_frame_id = self._ORIGINAL_FRAME_ID
            scale = self._image_RGB._scale
            
        if self.centers is None:
            tomatoRow = []
            tomatoCol = []
            radii = []
            mask = mask_empty
            
        else:
            xy = self.get_xy(self.centers, target_frame_id) 
            radii = self.radii/scale
            
            tomatoPixel = np.around(xy/scale).astype(int)
            tomatoRow = tomatoPixel[:, 1]
            tomatoCol = tomatoPixel[:, 0]
        
            mask = add_circles(mask_empty, tomatoPixel, radii, color = (255), thickness = -1)  
        
        tomato= {"row": tomatoRow, "col": tomatoCol, "radii": radii, "mask": mask, "pixel": tomatoPixel}
        return tomato


    def get_peduncle_image(self, local = False):
        if local:
            return self.crop(self._peduncle).get_data()
        else:
            return self._peduncle.get_data()
        
    def get_main_peduncle_image(self, local = False):
        if local:
            return self.penduncle_main
        else:
            return self.rescale(self.penduncle_main)
        
        
    def get_peduncle(self, local = False):
        peduncle_mask = self.get_peduncle_image(local = local)
        penduncle_main = self.get_main_peduncle_image(local = local)
        peduncle = {"mask": peduncle_mask,
                    "mask_main": penduncle_main}
        
        return peduncle

    def get_grasp_location(self, local = False):
        # TODO fix for local frame
        if local:
            frame_id = self._LOCAL_FRAME_ID
            scale = 1
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            scale = self._image_RGB._scale
            
        xy = self.get_xy(self.grasp_point, frame_id)
        graspPixel = np.around(xy/scale).astype(int)
        row = graspPixel[0,1]
        col =  graspPixel[0,0]
        angle = self.grasp_angle
        
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

    def get_tomato_visualization(self, local = False):
        if local == True:
            xy = self.get_xy(self.centers, self._LOCAL_FRAME_ID)
            img = self.crop(self._image_RGB).get_data()
        elif local == False:
            xy = self.get_xy(self.centers, self._ORIGINAL_FRAME_ID)
            img = self._image_RGB.get_data()
        return add_circles(img, xy, self.radii)
        
    def get_rgb(self, local = False):
        if local:
            img = self.crop(self._image_RGB).get_data()
        else:
            img = self._image_RGB.get_data()
        return img

    def get_truss_visualization(self, local = False):
        if local:
            frame_id = self._LOCAL_FRAME_ID
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            
        xy_center = self.get_xy(self.centers, frame_id)
        xy_grasp = self.get_xy(self.grasp_point, frame_id)
        img_rgb = self.get_rgb(local = local)        
        img_peduncle = self.get_peduncle_image(local = local)
        
        img = add_circles(img_rgb, xy_center, self.radii)
        img = add_contour(img, img_peduncle)       # 
        img = add_circles(img, xy_grasp, 20, color = (255, 0, 0), thickness = -1)
        return img 

    def get_segments(self, local = False):
        if local:
            tomato = self.crop(self._tomato).get_data()
            peduncle = self.crop(self._peduncle).get_data()
            background = self.crop(self._background).get_data()

        else:
            tomato = self._tomato.get_data()
            peduncle = self._peduncle.get_data()
            background = self._background.get_data()
            
        return tomato, peduncle, background

    def get_segmented_image(self, local = False):
        tomato, peduncle, background = self.get_segments(local = local)
        image_RGB = self.get_rgb(local = local)
        data = stack_segments(image_RGB, background, tomato, peduncle)
        return data

    def get_color_components(self):
        return self._image_hue, self._image_saturation, self._image_A

    def save_results(self, step, local = False):
        tomato, peduncle, background = self.get_segments(local = local)
        segments_rgb = self.get_segmented_image(local = local)
            
        save_img(background, self.pwdProcess, step + '_a', saveFormat = self.saveFormat)
        save_img(tomato, self.pwdProcess, step + '_b', saveFormat = self.saveFormat)
        save_img(peduncle, self.pwdProcess, step + '_c',  saveFormat = self.saveFormat) # figureTitle = "Peduncle",
        save_img(segments_rgb, self.pwdProcess, step + '_d', saveFormat = self.saveFormat)

    def process_image(self):

        self.color_space()
        
        success = self.segment_truss()
        if success is False:
            return success        
        
        success = self.filter_img()
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
            
        success = self.detect_grasp_location(strategy = "cage")
        return success

    def get_settings(self):
        settings = ImageProcessingSettings()
        settings.tomato_radius_min.data = self.tomato_radius_min
        settings.tomato_radius_max.data = self.tomato_radius_max
        settings.tomato_distance_min.data = self.tomato_distance_min
        settings.dp.data = self.dp
        settings.param1.data = self.param1
        settings.param2.data = self.param2
        
        return settings


# %matplotlib inline
# %matplotlib qt

if __name__ == '__main__':
    #%%########
    ## Path ##
    ##########

    ## params ##
    # params
    N = 15               # tomato file to load
    nDigits = 3
    saveIntermediate = True

    pathCurrent = os.path.dirname(__file__)
    dataSet = "real_blue" # "tomato_rot"

    pwdTest = os.path.join("..") # "..", "..", ,  "taeke"

    pwdData = os.path.join(pathCurrent, pwdTest, "data", dataSet)
    pwdResults = os.path.join(pathCurrent, pwdTest, "results", dataSet)



    # create folder if required
    if not os.path.isdir(pwdResults):
        print("New data set, creating a new folder: " + pwdResults)
        os.makedirs(pwdResults)

    # general settings

    #%%#########
    ### Loop ###
    ############
    for iTomato in range(N, N + 1, 1):

        tomatoName = str(iTomato).zfill(nDigits)
        fileName = tomatoName + ".png" #  ".jpg" # 

        rgb_data, DIM = load_rgb(pwdData, fileName, horizontal = True)

        if saveIntermediate:
            save_img(rgb_data, pwdResults, '01')

        proces_image = ProcessImage(camera_sim = False,
                             use_truss = True,
                             tomatoName = tomatoName, 
                             pwdProcess = pwdResults, 
                             saveIntermediate = saveIntermediate)
                             
                             
        proces_image.add_image(rgb_data)
        success = proces_image.process_image()

        if success:
        
            visual = proces_image.get_truss_visualization()
            save_img(visual, pwdResults, '99')
            
            visual = proces_image.get_truss_visualization(local = True)
            save_img(visual, pwdResults, '99')
        # plot_circles(image.imRGB, image.graspL, [10], pwd = pwdDataProc, name = str(iTomato), fileFormat = 'png')
        # plot_circles(image.imRGB, image.graspL, [10], pwd = pwdProcess, name = '06')
        # plot_circles(image.imRGBR, image.graspR, [10], pwd = pwdProcess, name = '06')
        
#        if success:         
#            plot_circles(rgb_data, proces_image.graspO, [10], pwd = pwdResults, name = '06')
#    
#            row, col, angle = proces_image.get_grasp_info()
#    
#            print('row: ', row)
#            print('col: ', col)
#            print('angle: ', angle)


