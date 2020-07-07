# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

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
# from flex_grasp.msg import ImageProcessingSettings

import skan

from image import Image, compute_angle, add, compute_bbox, image_rotate, image_crop

# custom functions
from util import add_border
from util import remove_all_blobs, romove_blobs
from util import segmentation_truss_real, segmentation_tomato_real
from util import segmentation_truss_sim
from util import translation_rot2or
from util import add_circles, add_contour

# junctionm dtection
from util import prune_branches_off_mask
from util import get_node_coord
from util import get_center_branch


from util import save_img
from util import load_rgb
from util import stack_segments
from util import plot_circles

from point_2d import make_2d_transform, make_2d_point


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
            # angle = np.deg2rad(45)
        # rotate
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

    def detect_tomatoes(self):
        #%%##################
        ## Detect tomatoes ##
        #####################
        success = True
        
        tomato_crop = self.crop(self._tomato)
        peduncle_crop = self.crop(self._peduncle)
        truss_crop = add(tomato_crop, peduncle_crop)
    
    
        truss_blurred = cv2.GaussianBlur(truss_crop.get_data(), self.blur_size, 0)
        minR = self._w/self.tomato_radius_min
        maxR = self._w/self.tomato_radius_max
        minDist = self._w/self.tomato_distance_min 

        circles = cv2.HoughCircles(truss_blurred, cv2.HOUGH_GRADIENT, 
                                   self.dp, minDist, param1 = self.param1, 
                                   param2 = self.param2, minRadius=minR, 
                                   maxRadius=maxR) # [x, y, radius]

        if circles is None:
            warnings.warn("Failed to detect any circle!")
            com = None
            centers = None
            radii = None
            success = False
        else:
            centers =np.matrix(circles[0][:,0:2])
            radii = circles[0][:,2]
            
            iKeep = []
            N = centers.shape[0]
            for i in range(0, N):
             
                image_empty = np.zeros(truss_crop.get_dimensions(), dtype=np.uint8)
                mask = cv2.circle(image_empty,(centers[i,0], centers[i,1]), radii[i], 255, -1)            
                
                res = cv2.bitwise_and(truss_crop.get_data(), mask)
                pixels = np.sum(res == 255)
                total = np.pi*radii[i]**2

                if pixels/total > 0.5:
                    iKeep.append(i)
                    
            centers = centers[iKeep, :]
            radii = radii[iKeep]            
            

            # find CoM
            com = (radii**2) * centers/(np.sum(radii**2))
            # comR = comL + self.box[0:2]
            # comO = rot2or(comR, self.DIM, -np.deg2rad(self.angle))
    
            # centersR = centers + self._bbox[0:2]
            # centersO = rot2or(centersR, self._image_RGB.get_dimensions(), -self._angle) # np.deg2rad(
            center_points = []
            for center in centers:
                center_points.append(make_2d_point(self._LOCAL_FRAME_ID, (center[0,0], center[0,1])))
                
            com_point =  make_2d_point(self._LOCAL_FRAME_ID, (com[0,0], com[0,1]))

        self.com = com_point
        self.centers = center_points
        self.radii = radii

        if self.saveIntermediate:
            xy_local = self.get_xy(center_points, self._LOCAL_FRAME_ID)
            plot_circles(self.crop(self._image_RGB).get_data(), xy_local, radii, savePath = self.pwdProcess, saveName = '05_a')
            
#            xy_rotated = self.get_xy(center_points, self._ROTATED_FRAME_ID)
#            plot_circles(self.rotate(self._image_RGB).get_data(), xy_rotated, radii, savePath = self.pwdProcess, saveName = '05_a')
            
            xy_original = self.get_xy(center_points, self._ORIGINAL_FRAME_ID)
            plot_circles(self._image_RGB.get_data(), xy_original, radii, savePath = self.pwdProcess, saveName = '05_a')
             
        return success

    def detect_peduncle(self):
    
        success = True    
    
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.peduncle_element)
        # penduncleMain = cv2.morphologyEx(cv2.morphologyEx(self.peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
        peduncleL = self.crop(self._peduncle).get_data()
        penduncleMain = cv2.morphologyEx(peduncleL, cv2.MORPH_OPEN, kernel)

        # only keep largest area
        penduncleMain = romove_blobs(penduncleMain, self.imMax)
        self.penduncleMain = penduncleMain

        if self.saveIntermediate:
            # https://stackoverflow.com/a/56142875
            contours, hierarchy= cv2.findContours(penduncleMain, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            segmentPeduncle = self.crop(self._image_RGB).get_data()
            cv2.drawContours(segmentPeduncle, contours, -1, (0,255,0), 3)
            save_img(segmentPeduncle, self.pwdProcess, '05_b', saveFormat= self.saveFormat)

#            penduncleMain = cv2.erode(self.peduncle,kernel,iterations = 1)
#            save_img(penduncleMain, self.pwdProcess, '05_b1', saveFormat = self.saveFormat)
#
#            penduncleMain = cv2.dilate(penduncleMain,kernel,iterations = 1)
#            save_img(penduncleMain, self.pwdProcess, '05_b2', saveFormat = self.saveFormat)
            
        return success


    def detect_junction(self):

        # create skeleton image
        peduncleL = self.crop(self._peduncle).get_data()
        skeleton_img = skeletonize(peduncleL/self.imMax)
        
        # intiailize for skan
        skeleton = skan.Skeleton(skeleton_img)
        branch_data = skan.summarize(skeleton)
        
        # get all node coordiantes
        junc_node_coord, dead_node_coord = get_node_coord(branch_data, skeleton)
        
        b_remove = (skeleton.distances < self.distance_threshold) & (branch_data['branch-type'] == 1) 
        i_remove = np.argwhere(b_remove)[:,0]
        
        # prune dead branches
        skeleton_prune_img = skeleton_img.copy()
        
        # update skeleton
        for i in i_remove:
            
            px_coords = skeleton.path_coordinates(i).astype(int)
            
            for px_coord in px_coords:
                skeleton_prune_img[px_coord[0], px_coord[1]] = False
        
        ## closing
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        close_img = cv2.dilate(skeleton_prune_img.astype(np.uint8), kernel, iterations = 1)
        
        # skeletonize
        skeleton_img_2 = skeletonize(close_img)
        skeleton_prune = skan.Skeleton(skeleton_img_2)
        branch_data_prune = skan.summarize(skeleton_prune)
        
        # prune brnaches of main peduncle
        iKeep = prune_branches_off_mask(self.penduncleMain, branch_data_prune)
        branch_data_prune = branch_data_prune.loc[iKeep]
        
        # prune small branches
        iKeep = branch_data_prune['branch-distance'] > 2
        branch_data_prune = branch_data_prune.loc[iKeep]
        
        junc_node_coord, dead_node_coord = get_node_coord(branch_data_prune, skeleton_prune)
        junc_branch_center, dead_branch_center = get_center_branch(branch_data_prune, skeleton_img)
        
        self.junc_branch_center = junc_branch_center


        if self.saveIntermediate:
            plot_circles(self.crop(self._image_RGB).get_data(), junc_branch_center, 5, savePath = self.pwdProcess, saveName = '05_c')

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
                skeleton = skeletonize(self.penduncleMain/self.imMax)
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
            plot_circles(self.crop(self._image_RGB).get_data(), xy_local, 10, savePath = self.pwdProcess, saveName = '06')
            
            
            xy_local = self.get_xy(grasp_point, self._ORIGINAL_FRAME_ID)
            plot_circles(self._image_RGB.get_data(), xy_local, 10, savePath = self.pwdProcess, saveName = '06')
            
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

    def get_tomatoes(self, local = False):
        if local:
            mask_empty = np.zeros(self._image_RGB.get_dimensions(), np.uint8)        
        else:
            mask_empty = np.zeros((self._h, self._w), np.uint8)        
            
        if self.centers is None:
            tomatoRow = []
            tomatoCol = []
            radii = []
            mask = mask_empty
            
        else:
            if local:   
                xy = self.get_xy(self.centers, self._LOCAL_FRAME_ID)
                scale = 1
            else:
                xy = self.get_xy(self.centers, self._ORIGINAL_FRAME_ID)            
                scale = self._image_RGB._scale
                
            radii = self.radii/scale
            
            tomatoPixel = np.around(xy/scale).astype(int)
            tomatoRow = tomatoPixel[:, 1]
            tomatoCol = tomatoPixel[:, 0]
        
            mask = add_circles(mask_empty, tomatoPixel, radii, color = (255), thickness = -1)  
        
        tomato= {"row": tomatoRow, "col": tomatoCol, "radii": radii, "mask": mask, "pixel": tomatoPixel}
        return tomato


    def get_peduncle_image(self, local = False):
        
        if local:
            return self._peduncle.get_data()
        else:
            return self.crop(self._peduncle).get_data()
        
        
    def get_peduncle(self):
        peduncle = {"mask": self._peduncle.get_data(),
                    "mask_main": self.penduncleMain}
        
        return peduncle

    def get_grasp_location(self):
        scale = self._image_RGB._scale
        xy = self.get_xy(self.grasp_point, self._ORIGINAL_FRAME_ID)
        graspPixel = np.around(xy/scale).astype(int)
        row = graspPixel[0,1]
        col =  graspPixel[0,0]
        angle = self.grasp_angle
        
#        print "grasp pixel: ", graspPixel
#        print "row: ", row
#        print "col: ", col
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
        
    def get_image(self, local = False):
        if local == True:
            img = self.crop(self._image_RGB).get_data()
        elif local == False:
            img = self._image_RGB.get_data()
        return img

    def get_truss_visualization(self):
        xy_center = self.get_xy(self.centers, self._ORIGINAL_FRAME_ID)
        xy_grasp = self.get_xy(self.grasp_point, self._ORIGINAL_FRAME_ID)
        
        img = add_circles(self._image_RGB.get_data(), xy_center, self.radii)
        img = add_contour(img, self._peduncle.get_data())       # self.rescale(self.penduncleMain)
        img = add_circles(img, xy_grasp, 20, color = (255, 0, 0), thickness = -1)
        return img       
        
        
    def get_segments(self, local = False):
        if local:
            return self.crop(self._tomato).get_data(), self.crop(self._peduncle).get_data(), self.crop(self._background).get_data()
        else:
            return self._tomato.get_data(), self._peduncle.get_data(), self._background.get_data()
            
        
        
    def get_segmented_image(self, local = False):
        data = stack_segments(self._image_RGB.get_data(), self._background.get_data(), self._tomato.get_data(), self._peduncle.get_data())
        if local:
            image = Image(data) 
            return self.crop(image).get_data()
        else:
            return data
        
    def get_color_components(self):
        return self._image_hue, self._image_saturation, self._image_A

    def crop(self, image):
        return image_crop(image, angle = -self._angle, bbox = self._bbox)
        
    def rotate(self, image):
        return image_rotate(image, angle = -self._angle)

    def rescale(self, img):
        
        # tomatoFilteredR= np.uint8(self.imMax*rotate(self.tomato, -angle, resize=True))
        imgR = np.uint8(self.imMax*rotate(img, self.angle, resize=True))
        return add_border(imgR, self.originO - self.originL, self.DIM)


    def save_results(self, step, crop = False):
        
        if crop:
            background = image_crop(self._background, angle=-self._angle, bbox=self._bbox).get_data()
            tomato = image_crop(self._tomato, angle=-self._angle, bbox=self._bbox).get_data()
            peduncle = image_crop(self._peduncle, angle=-self._angle, bbox=self._bbox).get_data()
            image_RGB = image_crop(self._image_RGB, angle=-self._angle, bbox=self._bbox).get_data()
        else:
            background = self._background.get_data()
            tomato = self._tomato.get_data()
            peduncle = self._peduncle.get_data()
            image_RGB = self._image_RGB.get_data()
            
        save_img(background, self.pwdProcess, step + '_a', saveFormat = self.saveFormat) # figureTitle = "Background",
        save_img(tomato, self.pwdProcess, step + '_b', saveFormat = self.saveFormat) # figureTitle = "Tomato",
        save_img(peduncle, self.pwdProcess, step + '_c',  saveFormat = self.saveFormat) # figureTitle = "Peduncle",

        segmentsRGB = stack_segments(image_RGB, background, tomato, peduncle)
        save_img(segmentsRGB, self.pwdProcess, step + '_d', saveFormat = self.saveFormat)

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
        
        success1 = self.detect_tomatoes()
        success2 = self.detect_peduncle()
        
        success = success1 and success2
        
        if success is False:
            return success
            
        self.detect_junction()
            
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

        features = proces_image.get_object_features()
        features["tomato"]['mask']
        
        visual = proces_image.get_truss_visualization()
        # plot_circles(image.imRGB, image.graspL, [10], savePath = pwdDataProc, saveName = str(iTomato), fileFormat = 'png')
        # plot_circles(image.imRGB, image.graspL, [10], savePath = pwdProcess, saveName = '06')
        # plot_circles(image.imRGBR, image.graspR, [10], savePath = pwdProcess, saveName = '06')
        
#        if success:         
#            plot_circles(rgb_data, proces_image.graspO, [10], savePath = pwdResults, saveName = '06')
#    
#            row, col, angle = proces_image.get_grasp_info()
#    
#            print('row: ', row)
#            print('col: ', col)
#            print('angle: ', angle)


