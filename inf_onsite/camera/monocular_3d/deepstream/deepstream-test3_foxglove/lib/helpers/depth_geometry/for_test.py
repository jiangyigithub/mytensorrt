# =============================================================================
#   C O P Y R I G H T                                     
# _____________________________________________/\\\\\\\\\\\_____/\\\\\\\\\_____/\\\\\\\\\\\\____        
#   Copyright (c) 2021 by Robert Bosch GmbH.  _\/////\\\///____/\\\\\\\\\\\\\__\/\\\////////\\\__       
#   All rights reserved.                       _____\/\\\______/\\\/////////\\\_\/\\\______\//\\\_      
#                                               _____\/\\\_____\/\\\_______\/\\\_\/\\\_______\/\\\_     
#   This file is property of Robert Bosch GmbH.  _____\/\\\_____\/\\\\\\\\\\\\\\\_\/\\\_______\/\\\_    
#   Any unauthorized copy or use or distribution  _____\/\\\_____\/\\\/////////\\\_\/\\\_______\/\\\_   
#   is an offensive act against international law  _____\/\\\_____\/\\\_______\/\\\_\/\\\_______/\\\__  
#   and may be prosecuted under federal law.        __/\\\\\\\\\\\_\/\\\_______\/\\\_\/\\\\\\\\\\\\/___ 
#   Its content is company confidential.             _\///////////__\///________\///__\////////////_____
# _______________________________________________________________________________________________________
#   P R O J E C T   I N F O R M A T I O N
# -----------------------------------------------------------------------------
#   IAD - Infrastructure-based Autonomous Driving (ICV-CN)
# =============================================================================


from os import listxattr
import cv2
import numpy as np
import math
from geopy import distance

trace_list = dict()
trace_list.update({'2':[[0,1,2],[0,11]]})
trace_list['2'][0].append(3)
#trace_list.update({'5':[1,2][1212]})
print(trace_list['2'][0][1])
print(trace_list['2'][0][2])
print(trace_list['2'][0][3])
def gps2xy(cam_gps,labels):
    x_enu = []  #radar cord x
    y_enu = []  #radar cord y
    bearing = []
    label_xy_enu = []
    lat_sp = math.radians(cam_gps[0])  # start point lat
    lon_sp = math.radians(cam_gps[1])  # start point lon
    for d in range(0, len(labels)):
        lat_ep = math.radians(labels[d][0])
        lon_ep = math.radians(labels[d][1])
        temp_b = math.atan2(
        math.sin(lon_ep - lon_sp) * math.cos(lat_ep),
            math.cos(lat_sp) * math.sin(lat_ep) -
            math.sin(lat_sp) * math.cos(lat_ep) * math.cos(lon_ep - lon_sp))
        temp_b = (math.pi * 2 + temp_b) % (math.pi * 2)
        bearing.append(temp_b)
        temp = distance.distance(cam_gps, labels[d]).km * 1000
        x_temp = temp * math.cos(math.pi / 2 - temp_b)
        y_temp = temp * math.sin(math.pi / 2 - temp_b)
        label_xy_enu.append([x_temp,y_temp]) 
        x_enu.append(x_temp)
        y_enu.append(y_temp)
    label_xy_enu = np.array(label_xy_enu).astype(np.float32)
    #label_xy = list(zip(y_enu, x_enu))
    return label_xy_enu

sourcePoints = [[429,600], [688,470], [1121,464], [1816,536]]# [[600,429], [470,688], [  464,1121], [ 536,1816]]
destinationPoints = [[ 26.04334716,13.16326991], [ 52.12611918,11.68603295], [ 49.23113245,-10.62360386], [ 31.07099815, -27.78000503]]
sourcePoints = np.float32([[c[0],c[1] ]for c in sourcePoints])
print(sourcePoints[0])
destinationPoints = np.float32([[c[0],c[1] ]for c in destinationPoints])
M = cv2.getPerspectiveTransform(sourcePoints,destinationPoints)
test_point =np.array([[[836,469]]], dtype = "float32")
test_point_1 =np.array([[[836,469]]], dtype = "float32")
img = cv2.imread('/home/nvidia/Pictures/test.png')
rows, cols = img.shape[:2]
dst_pt_1 = cv2.perspectiveTransform(test_point,M) 
img_1 = cv2.warpPerspective(img, M, (cols, rows)) 
res = cv2.warpPerspective(img, M, (cols, rows))



# cv2.imshow('2', img_1)

# cv2.waitKey(0)
test_xy = gps2xy([31.58638844, 120.4334113],[[31.58642061,120.4339596]])
print(test_xy)
print(M)
print(dst_pt_1)
