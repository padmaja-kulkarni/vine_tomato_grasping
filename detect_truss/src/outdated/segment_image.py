# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 12:44:23 2020

@author: taeke
"""
import numpy as np
import cv2

scale = 0.1

height = int(H * scale)
width = int(W * scale)
dim = (width, height)

RGB_mini = cv2.resize(imRGB, dim, interpolation = cv2.INTER_AREA)
im1_mini = cv2.resize(img_hue, dim, interpolation = cv2.INTER_AREA)    
data = np.stack((np.cos(np.deg2rad(2*np.float32(im1_mini.flatten()))), 
                 np.sin(np.deg2rad(2*np.float32(im1_mini.flatten())))), axis = 1)
                 
def hue_cirvular_hist():
    
    N = 180
    
    ## circular        
    width = (2*np.pi) / N
    bottom = 1
    theta = np.linspace(0.0, 2 * np.pi, N, endpoint=False)  
    
    # ax= plt.subplot(111, polar=True)
    fig, ax= plt.subplots(1)
    ax = plt.subplot(111, projection='polar')
    ax.set_rlim((0.1, 1000.0))
    ax.set_rscale('log')
    # bars = ax.bar(theta, radii, width=width, bottom=bottom)

def segmentation_truss_sim(img_saturation, img_hue, img_A, imMax):
    
        im0 = img_saturation    # saturation
        im1 = img_hue           # hue
        im2 = img_A             # A
 
        # Seperate robot from rest
        data0 = im0.flatten()
        thresholdRobot, temp = cv2.threshold(data0,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)    
        temp, robot = cv2.threshold(im0, thresholdRobot, imMax, cv2.THRESH_BINARY_INV)
        not_tobot = cv2.bitwise_not(robot)


        # Seperate truss from background
        data1 = im1[(not_tobot == imMax)].flatten()
        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, truss_1 = cv2.threshold(im1,thresholdTomato,imMax,cv2.THRESH_BINARY_INV)
        temp, truss_2 = cv2.threshold(im1,thresholdTomato + 90,imMax,cv2.THRESH_BINARY) # circle
        truss = cv2.bitwise_and(cv2.bitwise_or(truss_1,truss_2), not_tobot)
        background = cv2.bitwise_or(cv2.bitwise_not(truss), robot)
        
        # seperate tomato from peduncle
        data2 = im2[(truss == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(data2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
   
        temp, tomato_temp = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
        tomato = cv2.bitwise_and(tomato_temp, truss)
        peduncle = cv2.bitwise_and(cv2.bitwise_not(tomato_temp), truss)
        
        return background, tomato, peduncle


#def segmentation_truss_real(img_hue, imMax):
#        im1 = img_hue # hue
# 
#        # Seperate truss from background
#        data1 = im1.flatten()
#        thresholdTomato, temp = cv2.threshold(data1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#        
#        temp, truss_1 = cv2.threshold(im1,15,imMax,cv2.THRESH_BINARY_INV)
#        temp, truss_2 = cv2.threshold(im1,150,imMax,cv2.THRESH_BINARY) # circle
#        truss = cv2.bitwise_or(truss_1,truss_2)
#        background = cv2.bitwise_not(truss)
#        
#        # seperate tomato from peduncle
#        data2 = im1[(truss == imMax)].flatten()
#        threshPeduncle, temp = cv2.threshold(data2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#   
#
#        temp, peduncle_1 = cv2.threshold(im1,60,imMax,cv2.THRESH_BINARY_INV)
#        temp, peduncle_2 = cv2.threshold(im1,15,imMax,cv2.THRESH_BINARY)
#        peduncle = cv2.bitwise_and(peduncle_1, peduncle_2)
#        
#        return background, truss, peduncle

def segmentation_blue(imRGB, imMax):
        imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
        imLAB = cv2.cvtColor(imRGB, cv2.COLOR_RGB2LAB)
        
        #%%#################
        ### SEGMENTATION ###
        ####################
    
        im1 = imHSV[:, :, 1]
        im2 = imLAB[:, :, 1]
        

        # Otsu's thresholding
        thresholdTomato, temp = cv2.threshold(im2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        temp, tomato = cv2.threshold(im2,thresholdTomato,imMax,cv2.THRESH_BINARY)
        background = cv2.bitwise_not(tomato)
        

        # seperate tomato from peduncle
        dataCut = im1[(background == imMax)].flatten()
        threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#        
#        # label
        temp, peduncle = cv2.threshold(im1,threshPeduncle,imMax,cv2.THRESH_BINARY)
        peduncle = cv2.bitwise_and(peduncle, background)
        
        return background, tomato, peduncle

def segmentation_parametric(imRGB, imMax):
    
    # hsi segmentation
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    # plt.figure(), plt.imshow(imHSV)[:,:, 1] 
    
    a, background = cv2.threshold(imHSV[:,:,1], 0.4*imMax, imMax, cv2.THRESH_BINARY_INV)
    tomato = cv2.bitwise_not(background)
    # a, peduncle = cv2.threshold(imHSV[:,:,0], 0.1 * imMax, imMax, cv2.THRESH_BINARY_INV)
    peduncle = (imHSV[:,:,0] > 0.1*imMax) * tomato
    
    return background, tomato, peduncle


def segmentation_cluster(imRGB, imMax):
    
    # init
    [H, W] = imRGB.shape[:2]
    imHSI = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    
    imS = imHSI[:,:,1]
    data = np.float32(imS.reshape((H * W), 1))

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(data,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    # compute masks
    tomato = label.ravel() == np.argmax(center)
    tomato = tomato.reshape((H, W))
    tomato = bin2img(tomato) #convert to an unsigned byte
    tomato = romove_blobs_2(tomato, imMax)
    background = cv2.bitwise_not(tomato)
    

    
    # seperate tomato from peduncle
    imH = imHSI[:,:,0]
    data = np.float32(imH.reshape((H * W), 1))   
    dataCut = data[(tomato == 255).flatten()]
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(dataCut,2,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    
    
    # label
    peduncle = np.logical_and(tomato, imH > np.mean(center))
    peduncle = bin2img(peduncle)
    peduncle = romove_blobs(peduncle, imMax)
    
    
    # pixelLabel[peduncle == 255] = 2
    
    
    # background = bin2img(pixelLabel == 1)
    # tomato = bin2img(np.logical_or(pixelLabel == 0, pixelLabel == 2))
    # peduncle = bin2img(pixelLabel == 2)
    
    return background, tomato, peduncle
    
def segmentation_otsu(imRGB, imMax):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    imHSV = cv2.cvtColor(imRGB, cv2.COLOR_RGB2HSV)
    im1 = imHSV[:,:,1]
    im2 = imHSV[:,:,0]
    
    # init
    [h, w] = im1.shape[:2]
    data2 = im2.reshape((h * w), 1) 
    
    # Otsu's thresholding
    threshholdTruss,tomato = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    background = cv2.bitwise_not(tomato)
    
    # seperate tomato from peduncle
    dataCut = data2[(tomato == imMax).flatten()]
    threshPeduncle,peduncle = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
    peduncle = cv2.bitwise_and(tomato, peduncle)

    return background, tomato, peduncle

def segmentation_cluster_test(im1, im2, imMax, pwd, name):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    # init
    [h, w] = im1.shape[:2]
    data1 = np.float32(im1.reshape((h * w), 1))
    data2 = np.float32(im2.reshape((h * w), 1))   

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
    ret,label,center=cv2.kmeans(data = data1,
                                K = 2,
                                bestLabels = None,
                                criteria = criteria,
                                attempts = 10,
                                flags = cv2.KMEANS_RANDOM_CENTERS)
    
    # compute masks
    truss = label.ravel() == np.argmax(center)
    truss = truss.reshape((h, w))
    truss = bin2img(truss) 
    
    background_1 = cv2.bitwise_not(truss)
    
    #%%#################
    ### SECOND STAGE ###
    ####################
    K = 3
    labelSet = set(np.arange(0,K))
    dataCut = data2[(truss == 255).flatten()]
    centersInit = np.linspace(start = [0], stop = [256], num = K) # np.array([[0], [256/2], [256]])
    labelsInit = label_img(dataCut, centersInit)
    
    
    _, _, centers = cv2.kmeans(data = dataCut,
                                   K = K,
                                   bestLabels = labelsInit,
                                   criteria = criteria,
                                   attempts = 1,
                                   flags = cv2.KMEANS_USE_INITIAL_LABELS)
   
    # determine which center corresponds to which label
    peduncleLabel = np.argmin(centers)
    tomatoLabel = np.argmax(centers)
    
    if K == 3:
        backgroundLabel = list(labelSet - set( [peduncleLabel, tomatoLabel]))[0]   
    else:
        backgroundLabel = None
    # apply to entire image
    labels = label_img(data2, centers)
    
    tomato = label2img(labels, tomatoLabel, h ,w) 
    tomato = cv2.bitwise_and(tomato, truss)
    
    peduncle = label2img(labels, peduncleLabel, h ,w)
    peduncle = cv2.bitwise_and(peduncle, truss)
    
    if backgroundLabel is not None:
        background_2 = label2img(labels, backgroundLabel, h ,w)
        background = cv2.bitwise_or(background_1, background_2)
        truss = cv2.bitwise_not(background)
    else:
        background = background_1
    
    fig, ax= plt.subplots(1)
    ax.set_title('A (LAB)')
    ax.hist(dataCut.ravel(), bins=256, range=(0,255))
    ax.set_xlim(0, 255)
    ax.axvline(x=centers[peduncleLabel],  color='g')
    ax.axvline(x=centers[tomatoLabel],  color='r')
    
    if backgroundLabel is not None:
        ax.axvline(x=centers[backgroundLabel],  color='k')
    save_fig(fig, pwd, name + "_hist_2_k_means", figureTitle = "", resolution = 100, titleSize = 10)
    

    return background, tomato, peduncle,truss
    
def segmentation_otsu_test(im1, im2, imMax, pwd, name):
    # im1 is used for seperating background from the truss
    # im2 is used to seperate the tomato from the peduncle
    
    
  
    # init
    [h, w] = im1.shape[:2]
    data2 = im2.reshape((h * w), 1) 
    
    # Otsu's thresholding
    thresholdTruss, temp = cv2.threshold(im1,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    temp, truss = cv2.threshold(im1,thresholdTruss,imMax,cv2.THRESH_BINARY)
    background_1 = cv2.bitwise_not(truss)
    
    # seperate tomato from peduncle
    dataCut = data2[(truss == imMax).flatten()]
    threshPeduncle, temp = cv2.threshold(dataCut,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle_1 = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY_INV)
    temp, tomato = cv2.threshold(im2,threshPeduncle,imMax,cv2.THRESH_BINARY)
    peduncle_1 = cv2.bitwise_and(truss, peduncle_1)

    dataCut2 = data2[(peduncle_1 == imMax).flatten()]
    threshPeduncle_2, temp = cv2.threshold(dataCut2,0,imMax,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    # label
    temp, peduncle_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY_INV)
    temp, background_2 = cv2.threshold(im2,threshPeduncle_2,imMax,cv2.THRESH_BINARY)
    
    peduncle = cv2.bitwise_and(peduncle_2, peduncle_1)
    background = cv2.bitwise_or(background_2, background_1)
    # tomato = cv2.bitwise_and(cv2.bitwise_not(peduncle), cv2.bitwise_not(background))

    fig, ax= plt.subplots(1)
#    fig.suptitle('Histogram')
    
    ax.set_title('S (HSV)')
    values = ax.hist(im1.ravel(), bins=256/1, range=(0, 255))
    ax.set_ylim(0, 4*np.mean(values[0]))
    ax.set_xlim(0, 255)
    ax.axvline(x=thresholdTruss,  color='r')
    save_fig(fig, pwd, name + "_hist_1", figureTitle = "", resolution = 100, titleSize = 10)

    fig, ax= plt.subplots(1)
    ax.set_title('A (LAB)')
    values = ax.hist(dataCut.ravel(), bins=256/1, range=(0,255))
    ax.set_ylim(0, 4*np.mean(values[0]))
    ax.set_xlim(0, 255)
    ax.axvline(x=threshPeduncle,  color='r')
    ax.axvline(x=threshPeduncle_2,  color='r')
    save_fig(fig, pwd, name + "_hist_2", figureTitle = "", resolution = 100, titleSize = 10)

    return background, tomato, peduncle, truss