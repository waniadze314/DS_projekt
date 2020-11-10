#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 07:55:10 2020

@author: kamil
"""


import cv2 as cv
import numpy as np
from copy import copy

kernel = np.array([[0,-1,0,], [-1,4,-1],[0,-1,0]])
dilation_kernel = np.ones((5,5), np.uint8)
erode_kernel = np.ones((9,9), np.uint8)

low_thres = 125
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
    global dest, img_resized, color, detect
    dest = cv.inRange(color, low_thres, up_thres)
    dest = cv.erode(dest, dilation_kernel)
    dest = cv.dilate(dest, erode_kernel)
    edges = cv.filter2D(dest, -1, kernel)
    edges = cv.dilate(edges, dilation_kernel)
    detect = add_edge(img_resized, edges)
    cv.imshow("Edges", detect)
    
    
def on_change_lower(value):
    thres_filter(value, up_thres)
    
def on_change_upper(value):
    thres_filter(low_thres, value)

scale = 10
hsv = None

img = cv.imread('sample/defect/map_2.jpg')
width = int(img.shape[1]*scale/100)
height = int(img.shape[0]*scale/100)
dim = (width,height)
img_resized = cv.resize(img, dim)
img_copy = copy(img_resized)
hsv = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
# cv.imshow("test",hsv)
cv.namedWindow("Edges")
cv.createTrackbar("Lower threshold", "Edges", 125, 255, on_change_lower)
cv.createTrackbar("Upper threshold", "Edges", 180, 255, on_change_upper)
color, saturation, value = cv.split(hsv)
thres_filter(low_thres, up_thres)
cv.imshow("Edges", detect)
while True:
    if cv.waitKey(100)==27:
        break
cv.destroyAllWindows()