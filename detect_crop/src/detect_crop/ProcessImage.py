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
from matplotlib import pyplot as plt
from skimage.measure import label, regionprops
from skimage.transform import rotate
from skimage.morphology import skeletonize


# from skan import skeleton_to_csgraph
# from skan import Skeleton, summarize

# custom functions
from util import add_border
from util import romove_blobs
from util import segmentation_otsu

from util import rot2or
from util import or2rot

from util import save_fig
from util import load_rgb
from util import stack_segments
from util import plot_circles



class ProcessImage(object):
    
    def __init__(self, imRGB, tomatoName = 'tomato', saveIntermediate = False, pwdProcess = ''):
        print "Storing visiual results in: ", pwdProcess
        
        
        self.DIM = imRGB.shape[:2]
        self.imRGB = imRGB
        self.saveIntermediate = saveIntermediate

        self.imMax = 255
        self.pwdProcess = pwdProcess
        self.tomatoName = tomatoName

        self.filterDiameterTom = 11
        self.filterDiameterPend = 5

        save_fig(self.imRGB, self.pwdProcess, '01')

    def segment_img(self):
        #%%#################
        ### segmentation ###
        ####################
        
        background, tomato, peduncle = segmentation_otsu(self.imRGB, self.imMax)
        self.background = background
        self.tomato = tomato
        self.peduncle = peduncle
        
        if self.saveIntermediate:
            save_fig(self.background, self.pwdProcess, '02_a', figureTitle = "Background")
            save_fig(self.tomato, self.pwdProcess, '02_b', figureTitle = "Tomato")
            save_fig(self.peduncle, self.pwdProcess, '02_c', figureTitle = "Peduncle")
            
            segmentsRGB = stack_segments(self.imRGB, background, tomato, peduncle)
            save_fig(segmentsRGB, self.pwdProcess, '02')
        

    def filter_img(self):
        #%%###########
        ### Filter ###
        ##############
        
        # tomato
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.filterDiameterTom, self.filterDiameterTom))
        tomatoFiltered = cv2.morphologyEx(cv2.morphologyEx(self.tomato, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
        
        # peduncle
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.filterDiameterPend, self.filterDiameterPend))
        peduncleFiltered = cv2.morphologyEx(cv2.morphologyEx(self.peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
        peduncleFiltered = romove_blobs(peduncleFiltered, self.imMax)
        
        # background
        backgroundFiltered = cv2.bitwise_not(tomatoFiltered)
        
        self.background = backgroundFiltered
        self.tomato = tomatoFiltered
        self.peduncle = peduncleFiltered        
        
        if self.saveIntermediate:
           save_fig(backgroundFiltered, self.pwdProcess, '03_a', figureTitle = "Background")
           save_fig(tomatoFiltered, self.pwdProcess, '03_b', figureTitle = "Tomato")
           save_fig(peduncleFiltered, self.pwdProcess, '03_c', figureTitle = "Peduncle")
            
           segmentsRGB = stack_segments(self.imRGB, backgroundFiltered, tomatoFiltered, peduncleFiltered)
           save_fig(segmentsRGB, self.pwdProcess, '03')
           self.segmentsRGB = segmentsRGB    
           
    def rotate_cut_img(self):
        #%%###################
        ### rotate and cut ###
        ######################
        label_img = label(self.peduncle)
        regions = regionprops(label_img)
        if len(regions) > 1: warnings.warn("Multiple regions found!")
        angle = regions[0].orientation*180/np.pi
        
        # rotate
        tomatoFilteredR= np.uint8(self.imMax*rotate(self.tomato, -angle, resize=True))
        peduncleFilteredR = np.uint8(self.imMax*rotate(self.peduncle, -angle, resize=True))
        
        # get bounding box
        box = cv2.boundingRect(tomatoFilteredR)
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        
        # cut
        tomatoFilteredL = tomatoFilteredR[y:y+h, x:x+w]
        peduncleFilteredL = peduncleFilteredR[y:y+h, x:x+w]
        
        #get origin
        originR = np.matrix((x, y))
        originO = rot2or(originR, self.DIM, -angle/180*np.pi)
        
        self.tomato = tomatoFilteredL
        self.peduncle = peduncleFilteredL    
        self.box = box
        self.w = w
        self.h = h
        self.angle = angle
        
        self.originO = originO
        
        if self.saveIntermediate:
            backgroundFilteredR = np.uint8(self.imMax*rotate(self.background, -angle, resize=True))
            
            
            segmentsRGBR = rotate(self.segmentsRGB, -angle, resize=True)
            segmentsRGBL = segmentsRGBR[y:y+h, x:x+w, :]
            
            save_fig(tomatoFilteredL, self.pwdProcess, '04_a', figureTitle = "Tomato")
            save_fig(peduncleFilteredL, self.pwdProcess, '04_b', figureTitle = "Peduncle")
            
            save_fig(segmentsRGBL, self.pwdProcess, '04_c')
            
            imRGBR  = np.uint8(self.imMax*rotate(self.imRGB, -angle, resize=True))
            imRGBL = imRGBR[y:y+h, x:x+w, :]
            save_fig(imRGBL, self.pwdProcess, '04_d', saveFormat = 'png')
            
            
            self.segmentsRGBL = segmentsRGBL
            # plt.figure(), plt.imshow(tomatoFiltered), plt.title("tomato original")
            # plt.plot(originO[0], originO[1], 'bo')
        
            # plt.figure(), plt.imshow(tomatoFilteredR), plt.title("tomato rotated")
            # plt.plot(originR[0], originR[1], 'bo')

    def detect_tomatoes(self):       
        #%%##################
        ## Detect tomatoes ##
        #####################
        tomatoFilteredLBlurred = cv2.GaussianBlur(self.tomato, (5, 5), 0)    
        minR = self.w/8 # 6
        maxR = self.w/5
        minDist = self.w/5
        
        circles = cv2.HoughCircles(tomatoFilteredLBlurred, cv2.HOUGH_GRADIENT, 5, minDist, 
                                   param1=50,param2=100, minRadius=minR, maxRadius=maxR)
        
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
        self.radii = radii
        
        if self.saveIntermediate:
             plot_circles(self.segmentsRGBL, centersL, radii, savePath = self.pwdProcess, saveName = '05_a')

    def detect_peduncle(self):
        #%%##################
        ## DETECT PEDUNCLE ##
        #####################
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 2))
        penduncleMain = cv2.morphologyEx(cv2.morphologyEx(self.peduncle, cv2.MORPH_OPEN, kernel),cv2.MORPH_CLOSE, kernel)
        
        # only keep largest area
        penduncleMain = romove_blobs(penduncleMain, self.imMax)
        self.penduncleMain = penduncleMain
        
        if self.saveIntermediate:
            # https://stackoverflow.com/a/56142875
            contours, hierarchy= cv2.findContours(penduncleMain, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            segmentPeduncle = self.segmentsRGBL.copy()
            cv2.drawContours(segmentPeduncle, contours, -1, (0,255,0), 3)
            save_fig(segmentPeduncle, self.pwdProcess, '05_b')
        #plt.figure(), plt.imshow(penduncleMain)

    def detect_junction(self):
        #%%##################
        ## DETECT JUNCTION ##
        #####################
        
        skeleton = skeletonize(self.peduncle/255)
        pixel_graph0, coordinates0, degrees0 = skeleton_to_csgraph(skeleton)    
        
    
        branch_data = summarize(Skeleton(skeleton))
        branch_data.head()
    
        allJunctions = branch_data['node-id-src'].values
        deadBranch = branch_data['branch-type'] == 1
        junstionSrc = branch_data['node-id-src'][deadBranch].values
        junctionDst = branch_data['node-id-dst'][deadBranch].values
        
        
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
            plot_circles(self.segmentsRGBL, locMat, radiiJunction, savePath = self.pwdProcess, saveName = '05_c')

    def detect_grasp_location(self):

        #%%###################
        ### GRASP LOCATION ###
        ######################
        skeleton = skeletonize(self.penduncleMain/self.imMax)
        
        
        col, row = np.nonzero(skeleton)
        loc = np.transpose(np.matrix(np.vstack((row, col))))
        dist = np.sqrt(np.sum(np.power(loc - self.comL, 2), 1))
        iMin = np.argmin(dist)
        
        graspL = loc[iMin, :]
        graspR = graspL + self.box[0:1]
        graspO = rot2or(graspR, self.DIM, -self.angle/180*np.pi)
        
        self.graspO = np.around(graspO[0]).astype(int)
        
        if self.saveIntermediate:
            plot_circles(self.segmentsRGBL, graspL, [10], savePath = self.pwdProcess, saveName = '06')

    def show_process(self):
        #%%############
        ### RESCALE ###
        ###############
        originL = or2rot(np.matrix((1,1)), self.dim, self.angle/180*np.pi)
        
        # rotate
        tomatoFilteredR = np.uint8(self.imMax*rotate(self.tomato, self.angle, resize=True))
        peduncleFilteredR = np.uint8(self.imMax*rotate(self.peduncle, self.angle, resize=True))
        penduncleMainR = np.uint8(self.imMax*rotate(self.penduncleMain, self.angle, resize=True))
        
        tomatoOriginal = add_border(tomatoFilteredR, self.originO - originL , self.DIM)
        peduncleOriginal = add_border(peduncleFilteredR, self.originO - originL, self.DIM);
        penduncleMainOriginal = add_border(penduncleMainR, self.originO - originL, self.DIM);
    
        image =cv2.merge((tomatoOriginal, peduncleOriginal, penduncleMainOriginal))
        # image = cv2.cvtColor(total, cv2.COLOR_LAB2RGB)
    
        #%%##############
        ### VISUALIZE ###
        #################          
        fig = plt.figure()
        plt.subplot(2, 2, 1)
        plt.imshow(self.imRGB)
        
        plt.subplot(2, 2, 2)
        plt.imshow(image)
        
        
        plt.subplot(2, 2, 3)
        ax = fig.gca()
        
        if self.centersO is not None:
            for i in range(0, len(self.centersO[0]), 1):
                # draw the outer circle
                circle = plt.Circle((self.centersO[i, 0], self.centersO[i, 1]), self.radii[i], color='r')
                ax.add_artist(circle)
        
        
        circle = plt.Circle((self.comO[0, 0], self.comO[0, 1]), 10, color='b')
        ax.add_artist(circle)
        
        plt.imshow(self.imRGB)
        
        plt.subplot(2, 2, 4)
        plt.imshow(penduncleMainOriginal)
        
        
        fig.savefig(os.path.join(self.pwdResults, self.tomatoName), dpi = 300)


    def process_image(self):
        
        self.segment_img()
        self.filter_img()
        self.rotate_cut_img()
        
        self.detect_tomatoes()
        self.detect_peduncle()
        # self.detect_junction()

        self.detect_grasp_location()
    
def main():
    #%%########
    ## Path ##
    ##########
    
    ## params ##
    # params
    N = 11               # tomato file to load
    nDigits = 3   
    saveIntermediate = False
    
    pathCurrent = os.path.dirname(__file__)
    dataSet = "tomato_rot" # "tomato_rot"
    
    pwdTest = os.path.join("..", "..", "test")
    
    pwdData = os.path.join(pathCurrent, pwdTest, "data", dataSet)
    pwdDataProc = os.path.join(pathCurrent, pwdTest,"data_processed", dataSet)
    pwdResults = os.path.join(pathCurrent, pwdTest, "results", dataSet)
    
    
    # create folder if required
    if not os.path.isdir(pwdResults):
        print("New data set, creating a new folder: " + pwdResults)
        os.makedirs(pwdResults)
    
    if  not os.path.isdir(pwdDataProc):
        print("New data set, creating a new folder: " + pwdDataProc)
        os.makedirs(pwdDataProc)
        

    
    # general settings
    pwdProcess = os.path.join(pathCurrent, pwdTest, "results", "process")
    
    #%%#########
    ### Loop ###
    ############
    for iTomato in range(10, N, 1):
    
        tomatoName = "tomato" + "_RGB_" + str(iTomato).zfill(nDigits) 
        fileName = tomatoName + ".png" # png
        
        imRGB, DIM = load_rgb(pwdData, fileName, horizontal = True)
        
        if saveIntermediate:
            save_fig(imRGB, pwdProcess, '01')
            
        
        
        image = ProcessImage(imRGB, tomatoName = tomatoName, pwdProcess = pwdProcess, saveIntermediate = saveIntermediate)
        image.process_image()
        
        print image.graspO
    
if __name__ == '__main__':
    main()