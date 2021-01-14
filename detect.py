#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 07:55:10 2020

@author: kamil
"""


import cv2 as cv
import numpy as np
import easygui

kernel = np.array([[0,-1,0,], [-1,4,-1],[0,-1,0]])
erode_kernel = np.ones((5,5), np.uint8)
dilation_kernel = np.ones((9,9), np.uint8)

image_quality = 0;
low_thres = 126
up_thres = 180
dest = None
edges = None
detect = None

def chose_file():
    path=easygui.fileopenbox()
    return path

def add_edge(src, dest):
    blue, green, red = cv.split(src)
    red = cv.add(red, dest)
    out = cv.merge((blue,green,red))
    return out

def thres_filter(low_thres, up_thres):
    global dest, img_resized, color
    dest = cv.inRange(color, low_thres, up_thres)
    dest = cv.erode(dest, erode_kernel)
    dest = cv.dilate(dest, dilation_kernel)
    edges = cv.filter2D(dest, -1, kernel)
    detect = add_edge(img_resized, edges)
    return edges, detect
    
    
def on_change_lower(value):
    thres_filter(value, up_thres)
    
def on_change_upper(value):
    thres_filter(low_thres, value)
    
def find_black_rect(src, edges):
    global image_quality
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
        image_quality+=1
        return False
    
def check_correct_stich(src, edges):
    global image_quality
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
        image_quality+=1
    else:
        return True

def check_lumination(src):
    global image_quality
    lumin_thres = [25, 225]
    percent_thres = 5.0
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    sum = gray.shape[0]*gray.shape[1]
    histogram = cv.calcHist([gray], [0], None, [256], [0,256])
    bad_lumination_counter = 0
    for i in range(0,255):            
        percent_calc = 100*(histogram[i]/sum)
        if((i<=lumin_thres[0] or i >=lumin_thres[1]) and percent_calc > percent_thres):
            bad_lumination_counter=bad_lumination_counter+1    
    if(bad_lumination_counter>0):   
        image_quality+=1 
        return False
    else:
        return True

scale = 10
hsv = None
roi = None

img_path = chose_file()
img = cv.imread(img_path)
width = int(img.shape[1]*scale/100)
height = int(img.shape[0]*scale/100)
dim = (width,height)
img_resized = cv.resize(img, dim)
hsv = cv.cvtColor(img_resized, cv.COLOR_BGR2HSV)
color, saturation, value = cv.split(hsv)
edges, detect = thres_filter(low_thres, up_thres)
print("------------------------------------------------")
cv.imshow("Selected scan", detect)
if find_black_rect(img_resized, edges):
        print("Test1: No rectangle artifacts in image")
else:
        print("Test1: Rectangle artifacts in image")
if check_correct_stich(img_resized, edges):
    print("Test2: Image is stiched correctly")
else:
    print("Test2: Image stiching failed")
if check_lumination(img_resized):
    print("Test3: Lumination correct")
else:
    print("Test3: Bad lumination")
if(image_quality==0):
    print("Image is correct")
    flood = edges.copy()
    h,w = edges.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    cv.floodFill(flood, mask, (0,0), 255)
    flood_inverse = cv.bitwise_not(flood)
    roi = cv.merge((flood_inverse, flood_inverse, flood_inverse))
    output_scan = img_resized & roi
    cv.imshow("Extracted image", output_scan)

else:
    print("Image corrupted")
print("------------------------------------------------")
while True:
    if cv.waitKey(100)==27:
        break
cv.destroyAllWindows()