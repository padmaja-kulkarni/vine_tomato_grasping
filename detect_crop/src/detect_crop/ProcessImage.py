# -*- coding: utf-8 -*-


## imports ##
import os # os.sep
import warnings



import numpy as np
import tf2_ros
import cv2
import tf2_geometry_msgs
import json

from skimage.transform import rotate
import rospy

from image import Image, compute_angle, add, compute_bbox, image_rotate, image_crop, image_cut

from timer import Timer

# custom functions
from util import add_border

from util import translation_rot2or
from util import add_circles, add_contour

from util import make_dirs
from util import save_img
from util import load_rgb
from util import stack_segments
from util import plot_circles, plot_timer, save_fig
from util import change_brightness

from point_2d import make_2d_transform, make_2d_point

from matplotlib import pyplot as plt

from filter_segments import filter_segments
from detect_peduncle import detect_peduncle
from detect_tomato import detect_tomato, set_detect_tomato_settings
from segment_image import segment_truss, segment_tomato


warnings.filterwarnings('error', category=FutureWarning)

class ProcessImage(object):

    version = '0.1'

    # frame ids
    _ORIGINAL_FRAME_ID = 'original'
    _LOCAL_FRAME_ID = 'local'
    
    name_space = 'main'
    
    dtype = np.uint8

    def __init__(self, 
                 use_truss = True,
                 save = False, 
                 pwd = '', 
                 name = 'tomato', 
                 ext = 'png'               
                 ):

        self.ext = ext
        self.save = save
        self.use_truss = use_truss
        self.pwd = pwd
        self.name = name

        # image
        self.width_desired = 1280

        self.detect_tomato_settings = set_detect_tomato_settings()
        
        # detect junctions
        self.distance_threshold = 10

        # init buffer
        self.buffer_core = tf2_ros.BufferCore(rospy.Time(10))
    

    def add_image(self, data):
        image = Image(data)
        scale = image.rescale(self.width_desired)
        
        self.scale = scale
        self.image = image        
        self.shape = data.shape[:2]
        
        if self.save:
            save_img(self.image.data, self.pwd, '01', ext = self.ext)

    @Timer("color space", name_space)
    def color_space(self):
        self.image_hue = cv2.cvtColor(self.image.data, cv2.COLOR_RGB2HSV)[:, :, 0]
        # self.image_hue = imHSV[:, :, 0]

    @Timer("segment image", name_space)        
    def segment_image(self):

        success = True
        if self.use_truss:
            background, tomato, peduncle = segment_truss(self.image_hue, 
                                                         save = self.save, 
                                                         pwd = self.pwd, 
                                                         name = self.name)
        else:
            background, tomato, peduncle = segment_tomato(self.image_hue,
                                                          save = self.save, 
                                                          pwd = self.pwd, 
                                                          name = self.name)
        
        self.background = Image(background)
        self.tomato = Image(tomato)
        self.peduncle = Image(peduncle)
        

        if self.tomato.is_empty():
            warnings.warn("Segment truss: no pixel has been classified as tomato!")
            success = False
            
        if self.peduncle.is_empty():
            warnings.warn("Segment truss: no pixel has been classified as peduncle!")
            success = False

        if self.save:
            self.save_results('02')

        return success

    @Timer("filter image", name_space)
    def filter_img(self):

        tomato_f, peduncle_f, background_f = filter_segments(self.tomato.data, 
                                                              self.peduncle.data, 
                                                              self.background.data)

        self.tomato.data = tomato_f
        self.peduncle.data = peduncle_f
        self.background.data = background_f

        if self.save:
            self.save_results('03')
            
        if self.tomato.is_empty():
            warnings.warn("filter segments: no pixel has been classified as tomato")
            return False
            
        return True

    @Timer("crop image", name_space)
    def rotate_cut_img(self):
    
        if self.peduncle.is_empty():
            warnings.warn("Cannot rotate based on peduncle, since it does not exist!")
            angle = 0
        else:
            angle = compute_angle(self.peduncle.data) # [rad] 

        tomato_rotate = image_rotate(self.tomato, -angle)
        peduncle_rotate = image_rotate(self.peduncle, -angle)       
        truss_rotate = add(tomato_rotate, peduncle_rotate)
        
        if truss_rotate.is_empty():
            warnings.warn("Cannot crop based on tomato, since it does not exist!")
            return False

        bbox = compute_bbox(truss_rotate.data)
        x = bbox[0]
        y = bbox[1]

        #get origin
        translation = translation_rot2or(self.shape, -angle)
        dist = np.sqrt(translation[0]**2 + translation[1]**2)
 
        if angle >= 0:
            xy = (-x + dist,-y)
        if angle < 0:    
            xy = (-x,-y + dist)
        
        # make and add transform to the buffer
        transform = make_2d_transform(self._ORIGINAL_FRAME_ID, 
                                      self._LOCAL_FRAME_ID, 
                                      xy = xy, 
                                      angle = angle)             
                       
        self.buffer_core.set_transform(transform, "default_authority")
        self.bbox = bbox
        self.angle = angle
        
        self.tomato_crop = self.cut(tomato_rotate)
        self.peduncle_crop = self.cut(peduncle_rotate)
        self.image_crop = self.crop(self.image) 
        self.truss_crop = self.cut(truss_rotate) 
        
        if self.save:
            self.save_results('04', local = True)

    @Timer("detect tomatoes", name_space)
    def detect_tomatoes(self):
        success = True

        bg_img = change_brightness(self.image_crop.data, 0.85)
    
        centers, radii, com = detect_tomato(self.truss_crop.data, 
                                            self.detect_tomato_settings, 
                                            img_rgb=bg_img, 
                                            save = self.save, 
                                            pwd = self.pwd, 
                                            name = self.name)
            
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

    @Timer("detect peduncle", name_space)
    def detect_peduncle(self):
        distance_threshold = 10
        success = True    
    
        bg_img = change_brightness(self.image_crop.data, 0.85)

        mask, coord_center, junctions, ends  = detect_peduncle(self.peduncle_crop.data, 
                                                        distance_threshold, 
                                                        save = self.save, 
                                                        bg_img = bg_img, 
                                                        name = '06', 
                                                        pwd = self.pwd)
        # convert to 2D points
        junction_points = []
        for junction in junctions:
            junction_points.append(make_2d_point(self._LOCAL_FRAME_ID, junction))        

        # convert to 2D points
        end_points = []
        for end in ends:
            end_points.append(make_2d_point(self._LOCAL_FRAME_ID, end))       
        
        self.penduncle_main = mask
        self.junc_branch_center = coord_center
        self.junction_points = junction_points
        self.end_points = end_points
        return success

    @Timer("detect grasp location", name_space)
    def detect_grasp_location(self, strategy = 'cage'):
        success = True        

        if strategy== "cage":
            com = self.get_xy(self.com, self._LOCAL_FRAME_ID)
 
            if self.junc_branch_center.size > 0: # self.junc_branch_center:        
                loc = self.junc_branch_center
                dist = np.sqrt(np.sum(np.power(loc - com, 2), 1))
                
            else:
                print('Did not detect a junction')                
                # skeleton = skeletonize(self.penduncle_main/self.imMax)
                col, row = np.nonzero(self.penduncle_main)
                loc = np.transpose(np.matrix(np.vstack((row, col))))
                
                dist = np.sqrt(np.sum(np.power(loc - com, 2), 1))
                
            i = np.argmin(dist)
            grasp_angle = self.angle
            
        elif strategy== "pinch":
            
            # skeleton = skeletonize(self.peduncleL/self.imMax)
            col, row = np.nonzero(self.penduncle_main)
            loc = np.transpose(np.matrix(np.vstack((row, col))))
            
            dist0 = np.sqrt(np.sum(np.power(loc - self.centersL[0,:], 2), 1))
            dist1 = np.sqrt(np.sum(np.power(loc - self.centersL[1,:], 2), 1))
            dist = np.minimum(dist0, dist1)
            i = np.argmax(dist)
            
            grasp_angle = self.angle
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

        if self.save:
            xy_local = self.get_xy(grasp_point, self._LOCAL_FRAME_ID)
            plot_circles(self.crop(self.image).data, xy_local, 10, pwd = self.pwd, name = '06')
            
            
#            xy_local = self.get_xy(grasp_point, self._ORIGINAL_FRAME_ID)
#            plot_circles(self.image.data, xy_local, 10, pwd = self.pwd, name = '06')
            
        return success
        
    def transform_points(self, points, targer_frame_id):
        
        if isinstance(points, list):
            points_new = []
            for point in points:
                point_transform = self.buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id, rospy.Time(0)) 
                points_new.append(tf2_geometry_msgs.do_transform_pose(point, point_transform))
                
        else:
            point = points
            point_transform = self.buffer_core.lookup_transform_core(point.header.frame_id, targer_frame_id, rospy.Time(0)) 
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
        return image_crop(image, angle = -self.angle, bbox = self.bbox)
        
    def rotate(self, image):
        return image_rotate(image, angle = -self.angle)
        
    def cut(self, image):
        return image_cut(image, bbox = self.bbox)

    def rescale(self, img):
        
        # TODO: fix
        translation = translation_rot2or(self.shape, -self.angle)
        dist = np.sqrt(translation[0]**2 + translation[1]**2)
        x = self.bbox[0]
        y = self.bbox[0]
  
        if self.angle >= 0:
            transform = np.array(((-x + dist,-y),))
        if self.angle < 0:    
            transform = np.array(((-x,-y + dist),))
        
        imgR = rotate(img, self.angle, resize=True)
        return add_border(imgR, transform, self.shape)

    def get_tomatoes(self, local = False):
        if local:
            # shape = self.bbox[:2]      
            target_frame_id = self._LOCAL_FRAME_ID
            scale = 1.0
        else:
            # shape = self.shape
            target_frame_id = self._ORIGINAL_FRAME_ID
            scale = self.scale
            
        # mask_empty = np.zeros(shape, self.dtype)
            
        if self.centers is None:
#            row = []
#            col = []
            radii = []
            # mask = mask_empty
            centers = [[]]
            com = []
            
        else:
            centers_xy = self.get_xy(self.centers, target_frame_id) 
            com_xy = self.get_xy(self.com, target_frame_id)  
            radii = (self.radii/scale).astype(int).tolist()
            
            centers = np.around(centers_xy/scale).astype(int)
            com = np.around(com_xy/scale).astype(int)
#            row = centers[:, 1].tolist()
#            col = centers[:, 0].tolist()
            centers = centers.tolist()
            com = com.tolist()
#            centers = []
#            for row, col in zin(tomatoRow, tomatoCol):
#                center = [row, col]
#                centers.append(center)
        
#             mask = add_circles(mask_empty, centers, radii, color = (255), thickness = -1).tolist()
        
        tomato= {'centers': centers, 'radii': radii, 'com': com} # ,  "mask": mask, "row": row, "col": col
        return tomato


    def get_peduncle_image(self, local = False):
        if local:
            return self.crop(self.peduncle).data
        else:
            return self.peduncle.data
        
    def get_main_peduncle_image(self, local = False):
        if local:
            return self.penduncle_main
        else:
            return self.rescale(self.penduncle_main)
        
        
    def get_peduncle(self, local = False):
#        peduncle_mask = self.get_peduncle_image(local = local)
#        penduncle_main = self.get_main_peduncle_image(local = local)
#        peduncle = {"mask": peduncle_mask,
#                    "mask_main": penduncle_main}
    
        if local:
            frame_id = self._LOCAL_FRAME_ID
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            
        junction_points = self.get_xy(self.junction_points, frame_id).tolist()
        end_points = self.get_xy(self.end_points, frame_id).tolist()       
            
        peduncle = {'junctions': junction_points, 'ends': end_points}
        return peduncle

    def get_grasp_location(self, local = False):
        # TODO fix for local frame
        if local:
            frame_id = self._LOCAL_FRAME_ID
            scale = 1
        else:
            frame_id = self._ORIGINAL_FRAME_ID
            scale = self.scale
            
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
            img = self.crop(self.image).data
        elif local == False:
            xy = self.get_xy(self.centers, self._ORIGINAL_FRAME_ID)
            img = self.image.data
        return add_circles(img, xy, self.radii)
        
    def get_rgb(self, local = False):
        if local:
            img = self.crop(self.image).data
        else:
            img = self.image.data
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
            tomato = self.crop(self.tomato).data
            peduncle = self.crop(self.peduncle).data
            background = self.crop(self.background).data

        else:
            tomato = self.tomato.data
            peduncle = self.peduncle.data
            background = self.background.data
            
        return tomato, peduncle, background

    def get_segmented_image(self, local = False):
        tomato, peduncle, background = self.get_segments(local = local)
        image_RGB = self.get_rgb(local = local)
        data = stack_segments(image_RGB, background, tomato, peduncle)
        return data

    def get_color_components(self):
        return self.image_hue

    def save_results(self, step, local = False):
        tomato, peduncle, background = self.get_segments(local = local)
        segments_rgb = self.get_segmented_image(local = local)
            
        save_img(background, self.pwd, step + '_a', ext = self.ext)
        save_img(tomato, self.pwd, step + '_b', ext = self.ext)
        save_img(peduncle, self.pwd, step + '_c',  ext = self.ext) # figureTitle = "Peduncle",
        save_img(segments_rgb, self.pwd, step + '_d', ext = self.ext)

    @Timer("process image")
    def process_image(self):

        self.color_space()
        
        success = self.segment_image()
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
        return self.detect_tomato_settings
        
    def set_settings(self, settings):     
        settings = set_detect_tomato_settings(# blur_size = (3,3),
                               radius_min = settings['radius_min'],
                               radius_max = settings['radius_max'],
                               distance_min = settings['distance_min'], # = tomato_radius_max
                               dp = settings['dp'],
                               param1 = settings['param1'],
                               param2 = settings['param2'],
                               # ratio_threshold = 0.5
                               )

        self.detect_tomato_settings = settings

# %matplotlib inline
# %matplotlib qt

if __name__ == '__main__':
    i_start = 1
    i_end = 23
    N = i_end - i_start
    
    save = False

    pwd_current = os.path.dirname(__file__)
    dataset = "real_blue" # "tomato_rot"

    pwd_data = os.path.join(pwd_current, "..", "data", dataset)
    pwd_results = os.path.join(pwd_current, "..", "results", dataset)
    pwd_json = os.path.join(pwd_results, 'json')
    
    make_dirs(pwd_results)
    make_dirs(pwd_json)

    for count, i_tomato in enumerate(range(i_start, i_end)):
        print("Analyzing image %d out of %d" %(count + 1, N))

        tomato_name = str(i_tomato).zfill(3)
        file_name = tomato_name + ".png"

        rgb_data = load_rgb(pwd_data, file_name, horizontal = True)

        if save:
            save_img(rgb_data, pwd_results, '01')


        process_image = ProcessImage(use_truss = True,
                             name = tomato_name, 
                             pwd = pwd_results, 
                             save = save)                           
                             
        process_image.add_image(rgb_data)
        success = process_image.process_image()
        
        if False: # success:
            visual = process_image.get_truss_visualization()
            save_img(visual, pwd_results, '99')
            
            visual = process_image.get_truss_visualization(local = True)
            save_img(visual, pwd_results, '99')
            
            
        json_data = process_image.get_object_features()
        
        pwd_json_file = os.path.join(pwd_json, tomato_name + '.json')
        with open(pwd_json_file, "w") as write_file:
            json.dump(json_data, write_file)
            
    plot_timer(Timer.timers['main'].copy(), threshold = 0.02, pwd = pwd_results, name = 'main', title = 'Processing time')

    plot_timer(Timer.timers['peduncle'].copy(), N = N, threshold = 0.02, pwd = pwd_results, name = 'peduncle', title = 'Processing time peduncle')
    
    total_key = "process image"
    total = np.mean(Timer.timers[total_key])
    print(total)

    width = 0.5
    fig, ax = plt.subplots()
    
    time_ms = Timer.timers[total_key]
    time_s = [x / 1000 for x in time_ms]    
    
    ax.p1 = plt.bar(np.arange(i_start, i_end), time_s, width)
    
    plt.ylabel('time [s]')
    plt.xlabel('image ID')
    plt.title('Processing time per image')
    # plt.rc('ytick', labelsize=16)
    
    fig.show()    
    
    fig.savefig(os.path.join(pwd_results, 'time_bar'), dpi = 300) #, bbox_inches='tight', pad_inches=0)
    
    