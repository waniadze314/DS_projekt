#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 07:55:10 2020

@author: kamil
"""


import cv2 as cv
import numpy as np
from copy import copy
from matplotlib import pyplot as plt

kernel = np.array([[0,-1,0,], [-1,4,-1],[0,-1,0]])
dilation_kernel = np.ones((5,5), np.uint8)
erode_kernel = np.ones((9,9), np.uint8)

low_thres = 126
up_thres = 180
dest = None
edges = None
detect = None

def add_edge(src, dest):
    blue, green, red = cv.split(src)
    red = cv.add(red, dest)
    out = cv.merge((blue,green,red))
    return out

def thres_filter(low_thres, up_thres):
    global dest, img_resized, color
    dest = cv.inRange(color, low_thres, up_thres)
    dest = cv.erode(dest, dilation_kernel)
    dest = cv.dilate(dest, erode_kernel)
    edges = cv.filter2D(dest, -1, kernel)
    # edges = cv.dilate(edges, dilation_kernel)
    detect = add_edge(img_resized, edges)
    cv.imshow("Edges", detect)
    return edges, detect
    
    
def on_change_lower(value):
    thres_filter(value, up_thres)
    
def on_change_upper(value):
    thres_filter(low_thres, value)
    
def find_black_rect(src, edges):
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    sum = 0
    in_edge = None
    for i in range (0, gray.shape[0]):
        in_edge = False
        for j in range(0, gray.shape[1]):
            if (edges[i,j] == 255):
                if in_edge == False:
                    in_edge = True
                else:
                    in_edge = False
            if in_edge and gray[i,j] == 0:
                sum = sum+1          
    if sum <5:
        return True 
    else:
        return False
    
def check_correct_stich(src, edges):
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    sum = 0;
    artifacts_detected = 0
    stiching_edge_threshold = 120
    margin_y = 300
    critical_sum = margin_y*0.1
    in_edge = False
    for i in range(0, gray.shape[0]-margin_y):
        for j in range(0, gray.shape[1]-1):
            if (edges[i,j] == 255):
                if in_edge == False:
                    in_edge = True
                else:
                    in_edge = False
            if in_edge and (abs(int(gray[i,j])-int(gray[i,j+1]))>stiching_edge_threshold):
                sum = 0
                for x in range(i, i+margin_y):
                    if abs(int(gray[x,j])-int(gray[x,j+1]))>stiching_edge_threshold:
                        sum = sum + 1
                        if sum >= critical_sum:
                            artifacts_detected = artifacts_detected + 1
        in_edge = False 
    if artifacts_detected > 0:
        return False
    else:
        return True

scale = 10
hsv = None

img = cv.imread('sample/defect/map_2.jpg')
width = int(img.shape[1]*scale/100)
height = int(img.shape[0]*scale/100)
dim = (width,height)
img_resized = cv.resize(img, dim)
img_copy = copy(img_resized)
hsv = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
cv.namedWindow("Edges")
cv.createTrackbar("Lower threshold", "Edges", low_thres, 255, on_change_lower)
cv.createTrackbar("Upper threshold", "Edges", up_thres, 255, on_change_upper)
color, saturation, value = cv.split(hsv)
edges, detect = thres_filter(low_thres, up_thres)
if find_black_rect(img_resized, edges):
        print("No rectangle artifacts detected")
else:
        print("Image corrupted")
if check_correct_stich(img_resized, edges):
    print("Image is stiched correctly")
else:
    print("Image stiching failed")
# cv.imshow("img",img_resized )
# while True:
#     if cv.waitKey(100)==27:
#         break
cv.destroyAllWindows()