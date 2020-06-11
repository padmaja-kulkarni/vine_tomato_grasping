# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

## imports ##
import os # os.sep
import warnings


import numpy as np

import cv2

from skimage.measure import label, regionprops
from skimage.transform import rotate
from skimage.morphology import skeletonize


from skan import skeleton_to_csgraph
from skan import Skeleton, summarize

# custom functions
from util import add_border
from util import romove_blobs
from util import segmentation_truss_real, segmentation_tomato_real
from util import segmentation_truss_sim
from util import rot2or
from util import or2rot
from util import add_circles, add_contour



from util import save_img
from util import load_rgb
from util import stack_segments
from util import plot_circles



class ProcessImage(object):

    def __init__(self, imRGB, 
                 use_truss = True,
                 camera_sim = True, 
                 tomatoName = 'tomato', 
                 saveIntermediate = False, 
                 pwdProcess = '', 
                 saveFormat = 'png'):

        self.saveFormat = saveFormat

        DIM = imRGB.shape[:2]
        width_desired = 1280.0
        scale = width_desired/DIM[1]
        width = int(DIM[1] * scale)
        height = int(DIM[0] * scale)
        
        imRGB = cv2.resize(imRGB, (width, height), interpolation = cv2.INTER_AREA)
        self.scale = scale
        self.DIM = imRGB.shape[:2]
        self.W = DIM[0]
        self.H = DIM[1]        
        
        self.imRGB = imRGB
        self.saveIntermediate = saveIntermediate

        self.camera_sim = camera_sim
        self.use_truss = use_truss
        self.imMax = 255
        self.pwdProcess = pwdProcess
        self.tomatoName = tomatoName

        self.filterDiameterTom = 11
        self.filterDiameterPend = 5



        if self.saveIntermediate:
            save_img(self.imRGB, self.pwdProcess, '01', saveFormat = self.saveFormat)

    def color_space(self):
        
        imHSV = cv2.cvtColor(self.imRGB, cv2.COLOR_RGB2HSV)
        imLAB = cv2.cvtColor(self.imRGB, cv2.COLOR_RGB2LAB)
        self.img_hue = imHSV[:, :, 0]
        self.img_saturation = imHSV[:, :, 1]
        self.img_A  = imLAB[:, :, 1]
        
    def segment_truss(self):

        success = True
        if self.camera_sim:
            background, tomato, peduncle = segmentation_truss_sim(self.img_saturation, self.img_hue, self.img_A, self.imMax)

        else:
            if self.use_truss:
                background, tomato, peduncle = segmentation_truss_real(self.img_hue, self.imMax)
            else:
                background, tomato, peduncle = segmentation_tomato_real(self.img_A, self.imMax)
        
        self.background = background
        self.tomato = tomato
        self.peduncle = peduncle
        

        if np.all((tomato == 0)):
            warnings.warn("Segment truss: no pixel has been classified as tomato")
            success = False

        if self.saveIntermediate:
            self.save_results('02')

        return success

    def filter_img(self):
        #%%###########
        ### Filter ###
        ##############
        success = True

        # tomato
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.filterDiameterTom, self.filterDiameterTom))
        tomatoFiltered = cv2.morphologyEx(cv2.morphologyEx(self.tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)

        # peduncle
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.filterDiameterPend, self.filterDiameterPend))
        peduncleFiltered = cv2.morphologyEx(cv2.morphologyEx(self.peduncle, cv2.MORPH_CLOSE, kernel),cv2.MORPH_OPEN, kernel)
        peduncleFiltered = romove_blobs(peduncleFiltered, self.imMax)

        # background
        backgroundFiltered = cv2.bitwise_not(tomatoFiltered)

        self.background = backgroundFiltered
        self.tomato = tomatoFiltered
        self.peduncle = peduncleFiltered
        
        if np.all((tomatoFiltered == 0)):
            warnings.warn("filter segments: no pixel has been classified as tomato")
            success = False

        if self.saveIntermediate:
            self.save_results('03')
            
        return success

    def rotate_cut_img(self):
        #%%###################
        ### rotate and cut ###
        ######################
    
        if np.all((self.peduncle == 0)):
            warnings.warn("Cannot rotate based on peduncle, since it does not exist!")
    
        truss = cv2.bitwise_or(self.tomato, self.peduncle)
        label_img = label(truss)
        regions = regionprops(label_img , coordinates='xy')
            
        if len(regions) > 1: 
            warnings.warn("Multiple regions found!")
        angle = regions[0].orientation*180/np.pi # + 90
        # print('angle: ', angle)

        # rotate
        tomatoR= np.uint8(self.imMax*rotate(self.tomato, -angle, resize=True))
        peduncleR = np.uint8(self.imMax*rotate(self.peduncle, -angle, resize=True))
        backgroundR = np.uint8(self.imMax*rotate(self.background, -angle, resize=True))
        imRGBR  = np.uint8(self.imMax*rotate(self.imRGB, -angle, resize=True))

        trussR = cv2.bitwise_or(tomatoR, peduncleR)
        
        if np.all((trussR == 0)):
            warnings.warn("Cannot crop based on tomato, since it does not exist!")


        # get bounding box
        box = cv2.boundingRect(trussR)
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        dim = [h, w]
    
        # cut
        tomatoL = tomatoR[y:y+h, x:x+w]
        peduncleL = peduncleR[y:y+h, x:x+w]
        backgroundL = backgroundR[y:y+h, x:x+w]
        imRGBL = imRGBR[y:y+h, x:x+w, :]

        #get origin
        originR = np.matrix((x, y))
        originO = rot2or(originR, self.DIM, -angle/180*np.pi)
        originL = or2rot(np.matrix((1,1)), dim, angle/180*np.pi)

        self.backgroundL = backgroundL
        self.tomatoL = tomatoL
        self.peduncleL = peduncleL
        self.imRGBL = imRGBL
        # self.imRGBR = imRGBR

        self.box = box
        self.w = w
        self.h = h
        self.angle = angle

        self.originL = originL
        self.originO = originO
        self.originR = originR

        if self.saveIntermediate:
            self.save_results('04', local = True)
            save_img(self.imRGBL, self.pwdProcess, '04_e', saveFormat = self.saveFormat)


    def detect_tomatoes_global(self):
        tomatoFilteredLBlurred = cv2.GaussianBlur(self.tomato, (3, 3), 0)
        minR = self.W/20 # 6
        maxR = self.W/8
        minDist = self.W/10

        circles = cv2.HoughCircles(tomatoFilteredLBlurred, cv2.HOUGH_GRADIENT, 5, minDist,
                                   param1=50,param2=100, minRadius=minR, maxRadius=maxR)

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
    
        tomatoFilteredLBlurred = cv2.GaussianBlur(self.tomatoL, (1, 1), 0)
        minR = self.w/8 # 6
        maxR = self.w/4
        minDist = self.w/6

        circles = cv2.HoughCircles(tomatoFilteredLBlurred, cv2.HOUGH_GRADIENT, 5, minDist,
                                   param1=50,param2=100, minRadius=minR, maxRadius=maxR)

        if circles is None:
            warnings.warn("Failed to detect any circle!")
            comL = None
            comO = None
            centersO = None
            radii = None
            success = False
        else:
            centersL = np.matrix(circles[0][:,0:2])
            radii = circles[0][:,2]

            # find CoM
            comL = (radii**2) * centersL/(np.sum(radii**2))
            comR = comL + self.box[0:2]
            comO = rot2or(comR, self.DIM, -self.angle/180*np.pi)
    
            centersR = centersL + self.box[0:2]
            centersO = rot2or(centersR, self.DIM, -self.angle/180*np.pi)

        self.comL = comL
        self.comO = comO
        self.centersO = centersO
        self.centersL = centersL
        self.radii = radii

        if self.saveIntermediate:
             plot_circles(self.imRGBL, centersL, radii, savePath = self.pwdProcess, saveName = '05_a')
             
        return success

    def detect_peduncle(self):
        #%%##################
        ## DETECT PEDUNCLE ##
        #####################
    
        success = True    
    
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 2))
        # penduncleMain = cv2.morphologyEx(cv2.morphologyEx(self.peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
        penduncleMain = cv2.morphologyEx(self.peduncleL, cv2.MORPH_OPEN, kernel)

        # only keep largest area
        penduncleMain = romove_blobs(penduncleMain, self.imMax)
        self.penduncleMain = penduncleMain

        if self.saveIntermediate:
            # https://stackoverflow.com/a/56142875
            contours, hierarchy= cv2.findContours(penduncleMain, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            segmentPeduncle = self.imRGB.copy()
            cv2.drawContours(segmentPeduncle, contours, -1, (0,255,0), 3)
            save_img(segmentPeduncle, self.pwdProcess, '05_b', saveFormat= self.saveFormat)

            penduncleMain = cv2.erode(self.peduncle,kernel,iterations = 1)
            save_img(penduncleMain, self.pwdProcess, '05_b1', saveFormat = self.saveFormat)

            penduncleMain = cv2.dilate(penduncleMain,kernel,iterations = 1)
            save_img(penduncleMain, self.pwdProcess, '05_b2', saveFormat = self.saveFormat)
            
        return success



    def detect_junction(self):

        skeleton = skeletonize(self.peduncle/self.imMax)
        pixel_graph0, coordinates0, degrees0 = skeleton_to_csgraph(skeleton)


        branch_data = summarize(Skeleton(skeleton))
        branch_data.head()

        allJunctions = branch_data['node-id-src'].values
        deadBranch = branch_data['branch-type'] == 1 # junction-to-endpoint
        junstionSrc = branch_data['node-id-src'][deadBranch].values # from node
        junctionDst = branch_data['node-id-dst'][deadBranch].values # to node

        # Prune all nodes which correspond to a branch going from junction to an endpoint
        allJunctions = np.setdiff1d(allJunctions,junstionSrc)
        allJunctions = np.setdiff1d(allJunctions,junctionDst)

        locMat = coordinates0[allJunctions]
        # col, row = np.nonzero((degrees0 == 3) & (penduncleMain > 0))
        # loc = np.transpose(np.matrix(np.vstack((row, col))))
        locMat[:,[0, 1]] = locMat[:,[1, 0]]

        col, row = np.nonzero(self.penduncleMain)
        loc = np.transpose(np.matrix(np.vstack((row, col))))

        iKeep = []
        for i in range(locMat.shape[0]):
            junction = locMat[i,:]
            col, row = np.nonzero(skeleton)
            dist = np.sqrt(np.sum(np.power(loc - junction, 2), 1))
            if np.amin(dist) < 20:
                iKeep.append(i)

        locMat = locMat[iKeep, :]
        radiiJunction = np.repeat(5, locMat.shape[0])

        if self.saveIntermediate:
            plot_circles(self.imRGB, locMat, radiiJunction, savePath = self.pwdProcess, saveName = '05_c')

    def detect_grasp_location(self, strategy = 'cage'):
        success = True        

        if strategy== "cage":
            
            skeleton = skeletonize(self.penduncleMain/self.imMax)
            col, row = np.nonzero(skeleton)
            loc = np.transpose(np.matrix(np.vstack((row, col))))
            
            dist = np.sqrt(np.sum(np.power(loc - self.comL, 2), 1))
            i = np.argmin(dist)
            
            grasp_angle = self.angle
            
        elif strategy== "pinch":
            
            skeleton = skeletonize(self.peduncleL/self.imMax)
            col, row = np.nonzero(skeleton)
            loc = np.transpose(np.matrix(np.vstack((row, col))))
            
            dist0 = np.sqrt(np.sum(np.power(loc - self.centersL[0,:], 2), 1))
            dist1 = np.sqrt(np.sum(np.power(loc - self.centersL[1,:], 2), 1))
            dist = np.minimum(dist0, dist1)
            i = np.argmax(dist)
            
            grasp_angle = self.angle + 90
        else:
            print("Unknown grasping strategy")
            return False
            
        
        graspL = loc[i, :]
        graspR = graspL + [self.box[0], self.box[1]]
        graspO = rot2or(graspR, self.DIM, -self.angle/180*np.pi)

        self.graspL = graspL
        self.graspR = graspR
        self.graspO = graspO
        self.grasp_angle = grasp_angle

        if self.saveIntermediate:
            plot_circles(self.imRGBL, graspL, [10], savePath = self.pwdProcess, saveName = '06')
            
        return success

    def get_tomatoes(self):

        mask_empty = np.zeros((self.W, self.H), np.uint8)        
        
        if self.centersO is None:
            tomatoRow = []
            tomatoCol = []
            radii = []
            mask = mask_empty
            
        else:
            tomatoPixel = np.around(self.centersO/self.scale).astype(int)
            radii = self.radii/self.scale
            
            tomatoRow = tomatoPixel[:, 1]
            tomatoCol = tomatoPixel[:, 0]
        
            
            mask = add_circles(mask_empty, tomatoPixel, radii, color = (255), thickness = -1)  
        
        tomato= {"row": tomatoRow, "col": tomatoCol, "radii": radii, "mask": mask}
        return tomato
        
    def get_peduncle(self):
        peduncle = {"mask": self.peduncle,
                    "mask_main": self.penduncleMain}
        
        return peduncle

    def get_grasp_location(self):
        graspPixel = np.around(self.graspO[0]/self.scale).astype(int)
        row = graspPixel[1]
        col =  graspPixel[0]
        angle = self.grasp_angle/180*np.pi

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

    def get_grasp_info(self):
        graspPixel = np.around(self.graspO[0]).astype(int)

        row = graspPixel[1]
        col = graspPixel[0]
        angle = self.angle/180*np.pi

        return row, col, angle
        
    def get_tomato_visualization(self):
        return add_circles(self.imRGB, self.centersO, self.radii)
        
        
    def get_truss_visualization(self):
        img = add_circles(self.imRGB, self.centersO, self.radii)
        img = add_contour(img, self.peduncle)       # self.rescale(self.penduncleMain)
        img = add_circles(img, self.graspO, [20], color = (255, 0, 0), thickness = -1)
        return img       
        
        
    def get_segmented_image(self, local = False):
        
        if local:
            return stack_segments(self.imRGBL, self.backgroundL, self.tomatoL, self.peduncleL)
        else:
            return stack_segments(self.imRGB, self.background, self.tomato, self.peduncle)
        
    def get_color_components(self):
        return self.img_hue, self.img_saturation, self.img_A

    def rescale(self, img):
        
        # tomatoFilteredR= np.uint8(self.imMax*rotate(self.tomato, -angle, resize=True))
        imgR = np.uint8(self.imMax*rotate(img, self.angle, resize=True))
        return add_border(imgR, self.originO - self.originL, self.DIM)


    def save_results(self, step, local = False):
        
        if local:
            background = self.backgroundL
            tomato = self.tomatoL
            peduncle = self.peduncleL
        else:
            background = self.background
            tomato = self.tomato
            peduncle = self.peduncle
            
        save_img(background, self.pwdProcess, step + '_a', saveFormat = self.saveFormat) # figureTitle = "Background",
        save_img(tomato, self.pwdProcess, step + '_b', saveFormat = self.saveFormat) # figureTitle = "Tomato",
        save_img(peduncle, self.pwdProcess, step + '_c',  saveFormat = self.saveFormat) # figureTitle = "Peduncle",

        segmentsRGB = stack_segments(self.imRGB, self.background, self.tomato, self.peduncle)
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
        self.detect_junction()

        success = success1 and success2
        if success is False:
            return success
            
        success = self.detect_grasp_location(strategy = "pinch")
        return success

def main():
    #%%########
    ## Path ##
    ##########

    ## params ##
    # params
    N = 1               # tomato file to load
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

        imRGB, DIM = load_rgb(pwdData, fileName, horizontal = True)

        if saveIntermediate:
            save_img(imRGB, pwdResults, '01')



        image = ProcessImage(imRGB, 
                             camera_sim = False,
                             use_truss = True,
                             tomatoName = tomatoName, 
                             pwdProcess = pwdResults, 
                             saveIntermediate = saveIntermediate)
        success = image.process_image()

        # plot_circles(image.imRGB, image.graspL, [10], savePath = pwdDataProc, saveName = str(iTomato), fileFormat = 'png')
        # plot_circles(image.imRGB, image.graspL, [10], savePath = pwdProcess, saveName = '06')
        # plot_circles(image.imRGBR, image.graspR, [10], savePath = pwdProcess, saveName = '06')
        
        if success:         
            plot_circles(imRGB, image.graspO, [10], savePath = pwdResults, saveName = '06')
    
            row, col, angle = image.get_grasp_info()
    
            print('row: ', row)
            print('col: ', col)
            print('angle: ', angle)

if __name__ == '__main__':
    main()
