#!/usr/bin/env python3

################################################################################
# SPDX-FileCopyrightText: Copyright (c) 2019-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import sys
sys.path.append('../')
from pathlib import Path
import gi
import configparser
import argparse
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst
from ctypes import *
import time
import sys
import math
import platform
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.FPS import PERF_DATA

# from vision_msgs.msg import Classification2D, ObjectHypothesis, ObjectHypothesisWithPose, BoundingBox2D, Detection2D, Detection2DArray


import pyds

no_display = True
silent = False
file_loop = False
perf_data = None

fps_streams = {}
frame_count = {}
saved_count = {}
calibration = False

MAX_DISPLAY_LEN=640
PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3
MUXER_OUTPUT_WIDTH=1920
MUXER_OUTPUT_HEIGHT=1080
MUXER_BATCH_TIMEOUT_USEC=4000000
TILED_OUTPUT_WIDTH=1280
TILED_OUTPUT_HEIGHT=720
GST_CAPS_FEATURES_NVMM="memory:NVMM"
OSD_PROCESS_MODE= 0
OSD_DISPLAY_TEXT= 1
pgie_classes_str= ["Vehicle", "TwoWheeler", "Person","RoadSign"]

# pgie_src_pad_buffer_probe  will extract metadata received on tiler sink pad
# and update params for drawing rectangle, object information etc.
def pgie_src_pad_buffer_probe(pad,info,u_data):
    frame_number=0
    num_rects=0
    got_fps = False
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return
    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number=frame_meta.frame_num
        l_obj=frame_meta.obj_meta_list
        num_rects = frame_meta.num_obj_meta
        while l_obj is not None:
            print('---------------------------obj_detected---------------------------')
            try: 
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            try: 
                l_obj=l_obj.next
            except StopIteration:
                break

            print("class_id:   ",(obj_meta.class_id))
            print("confidence: ",'%.2f'%(obj_meta.confidence))
            print("left:       ",'%.2f'%(obj_meta.rect_params.left))
            print("top:        ",'%.4f'%(obj_meta.rect_params.top))
            print("width:      ",'%.2f'%(obj_meta.rect_params.width))
            print("height:     ",'%.6f'%(obj_meta.rect_params.height))

        # #Update frame rate through this probe
        stream_index = "stream{0}".format(frame_meta.pad_index)
        global perf_data
        perf_data.update_fps(stream_index)

        try:
            l_frame=l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK

def tiler_sink_pad_buffer_probe(pad,info,u_data):
        frame_number=0
        num_rects=0
        gst_buffer = info.get_buffer()
        if not gst_buffer:
            print("Unable to get GstBuffer ")
            return

        print('================================ Frame00 ====================================')

        # Retrieve batch metadata from the gst_buffer
        # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
        # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
        batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))

        l_frame = batch_meta.frame_meta_list
        frames = 0
        print('================================ Frame01 ====================================')
        while l_frame is not None:
            print('================================ Frame02 ====================================')
            try:
                # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
                # The casting is done by pyds.NvDsFrameMeta.cast()
                # The casting also keeps ownership of the underlying memory
                # in the C code, so the Python garbage collector will leave
                # it alone.
                frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
            except StopIteration:
                break

            frame_number=frame_meta.frame_num
            l_obj=frame_meta.obj_meta_list
            num_rects = frame_meta.num_obj_meta
            is_first_obj = True
            save_image = False
            # fps_v = fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()
            # print('=======FPS======',fps_v)
            # Message for output of detection inference
            # msg = Detection2DArray()
            # msg_rix = Objects()
            temp_data = []
            if calibration:
                temp_calibration =[]
            print('================================ Frame03 ====================================')
            while l_obj is not None:
                print('================================ Frame04 ====================================')
                try:
                    # Casting l_obj.data to pyds.NvDsObjectMeta
                    obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
                    l_classifier = obj_meta.classifier_meta_list
                    # If object is a car (class ID 0), perform attribute classification
                except StopIteration:
                    break

                ##print('----------------------------------')
                # print('l_obj: ', type(l_obj))
                # print('obj_meta: ', type(obj_meta))
                # print('obj_meta.class_id: ', type(obj_meta.class_id))

                # Creating message for output of detection inference
                # result = ObjectHypothesisWithPose()
                # result.id = str(self.class_obj[obj_meta.class_id])
                #print(self.class_obj)
                #print(self.class_obj[-1])
                #print(obj_meta.class_id)

                # print('class_id: [', obj_meta.class_id, "], Tracking_ID: [",obj_meta.object_id, "], confidence: [ %f",obj_meta.confidence, "]")
                # print('class_id: [', obj_meta.class_id, '], ', 'class_obj: ', class_obj[obj_meta.class_id])
                # result.score = obj_meta.confidence
                print('0000: the confidence is %f' %(obj_meta.confidence))

                left = obj_meta.rect_params.left
                top = obj_meta.rect_params.top
                width = obj_meta.rect_params.width
                height = obj_meta.rect_params.height
                print("---------+++++++++++++++-------------")
                print('left/top/width/height: [', int(left), int(top), int(width), int(height), ']')
                # print('[left/top/width/height/has_bg_color]: ', left, top, width, height, obj_meta.rect_params.has_bg_color)
                # obj_meta.rect_params.bg_color.green = 1.0
                # print('bg_color[r/g/b/alpha]: ', obj_meta.rect_params.bg_color.red, obj_meta.rect_params.bg_color.green, 
                # obj_meta.rect_params.bg_color.blue, obj_meta.rect_params.bg_color.alpha)

                # bounding_box = BoundingBox2D()
                # bounding_box.center.x = float(left + (width/2)) 
                # bounding_box.center.y = float(top + (height))
                # bounding_box.size_x = width
                # bounding_box.size_y = height
                
                # detection = Detection2D()
                # detection.results.append(result)
                # detection.bbox = bounding_box
                # detection.tracking_id = str(obj_meta.object_id)
                # msg.detections.append(detection)

                # pt = np.float32([[left + (width/2), top + (height)]]) # bottom center
                # pts = np.array([pt])
                # print('bottom center Pixel: [', int(pts[0][0][0]), ', ', int(pts[0][0][1]), ']')
                # Pixel_IOU_flag = True

                # Zc = projection_test.get_Zc(pts[0][0][0],pts[0][0][1])
                # tp_xy = projection_test.reverse_porjection(pts[0][0][0],pts[0][0][1],Zc)
                # tp_xy = tp_xy/1000
                # loc = [0, 0]
                # loc[0] = tp_xy[0]
                # loc[1] = tp_xy[1]
                # result.pose.pose.position.x = float(tp_xy[0])
                # result.pose.pose.position.y = float(tp_xy[1])
                # detection_rix = Object()
                # detection_rix.header.frame_id = frame_id
                # detection_rix.header.stamp = self.get_clock().now().to_msg() 
                # detection_rix.id = obj_meta.object_id# Tracking_id
                # detection_rix.position.x = float(tp_xy[0])
                # detection_rix.position.y = float(tp_xy[1])
                # detection_rix.existence_probability = 0.9
                # print('position: [', float(tp_xy[0]), ', ', float(tp_xy[1]), ']')
                
                # if result.id =='car':
                #     detection_rix.width = 1.5
                #     detection_rix.length = 3.
                #     #detection_rix.heigth = 2.
                #     if  (bounding_box.center.y<self.Vehicle_filter_y[0] or bounding_box.center.y>self.Vehicle_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Vehicle_filter_x[0] or bounding_box.center.x>self.Vehicle_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'car'
                # elif result.id =='bus':
                #     detection_rix.width = 2.5
                #     detection_rix.length = 8.
                #     #detection_rix.heigth = 1.5
                #     if  (bounding_box.center.y<self.Vehicle_filter_y[0] or bounding_box.center.y>self.Vehicle_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Vehicle_filter_x[0] or bounding_box.center.x>self.Vehicle_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'bus'
                # elif result.id =='truck':
                #     detection_rix.width = 2.5
                #     detection_rix.length = 8.
                #     #detection_rix.heigth = 1.5
                #     if  (bounding_box.center.y<self.Vehicle_filter_y[0] or bounding_box.center.y>self.Vehicle_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Vehicle_filter_x[0] or bounding_box.center.x>self.Vehicle_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'truck'
                # elif result.id =='bicycle' or result.id =='motorbike':
                #     detection_rix.width = 0.5
                #     detection_rix.length = 1.5
                #     #detection_rix.heigth = 1.5
                #     if  (bounding_box.center.y<self.Bicycle_filter_y[0] or bounding_box.center.y>self.Bicycle_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Bicycle_filter_x[0] or bounding_box.center.x>self.Bicycle_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'bicycle'
                # elif result.id =='person':
                #     detection_rix.width = 0.5
                #     detection_rix.length = 0.5
                #     #detection_rix.heigth = 2.
                #     if  (bounding_box.center.y<self.Person_filter_y[0] or bounding_box.center.y>self.Person_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Person_filter_x[0] or bounding_box.center.x>self.Person_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'person'
                # elif result.id =='barrier':
                #     detection_rix.width = 0.5
                #     detection_rix.length = 0.5
                #     #detection_rix.heigth = 2.
                #     if  (bounding_box.center.y<self.Barrier_filter_y[0] or bounding_box.center.y>self.Barrier_filter_y[1]) \
                #         or \
                #         (bounding_box.center.x<self.Barrier_filter_x[0] or bounding_box.center.x>self.Barrier_filter_x[1]) :
                #         Pixel_IOU_flag = False
                #     obj_type = 'barrier'
                # else:
                #     detection_rix.width = 0.5
                #     detection_rix.length = 0.5
                #     #detection_rix.heigth = 0.5
                #     obj_type = 'scissors'

                # classification = Classification()
                # cl = []
                # classification.obj_class = obj_type
                # classification.confidence = obj_meta.confidence
                # cl.append(classification)
                # detection_rix.classification = cl

                # print('type[', detection_rix.classification[-1].obj_class,  '], length/width= ', detection_rix.length, detection_rix.width,'confidence=',classification.confidence)

                # if obj_meta.object_id in id_list:
                #     loc = [0, 0]
                #     loc[0] = tp_xy[0]
                #     loc[1] = tp_xy[1]
                #     if trace_list[obj_meta.object_id] is None:
                #         trace_list[obj_meta.object_id] = loc
                #     else:
                #         trace_list[obj_meta.object_id].append(loc)
                #     # if frame_number[obj_meta.object_id] is None:
                #     #     frame_number[obj_meta.object_id] = frames
                #     # else:
                #     kalman_filter_list[str(obj_meta.object_id)].update(np.array([detection_rix.position.x,detection_rix.position.y]).reshape(2,1))
                #     # estimate the tracker velocity by using Kalman filter (update by Handai)
                #     if len(trace_list[obj_meta.object_id]) >=3:
                #         estimate = kalman_filter_list[str(obj_meta.object_id)].get_state()
                #         posi_x = estimate[0]
                #         posi_y = estimate[1]
                #         vel_xe = estimate[2]
                #         vel_ye = estimate[3]
                #         if math.sqrt(float(vel_xe)*float(vel_xe)+float(vel_ye)*float(vel_ye))*3.6 >= 0.5:
                #             detection_rix.velocity.x = float(vel_xe)
                #             detection_rix.velocity.y = float(vel_ye)
                #             detection_rix.position.x = float(posi_x)
                #             detection_rix.position.y = float(posi_y)
                #             # calculate heading
                #             ang = math.atan2(detection_rix.velocity.y, detection_rix.velocity.x)
                #             detection_rix.yaw = ang
                #         else:
                #             detection_rix.velocity.x = 0.01
                #             detection_rix.velocity.y = 0.01
                #             detection_rix.yaw = self.default_heading
                #         #print('speed: ', math.sqrt(float(vel_xe)*float(vel_xe)+float(vel_ye)*float(vel_ye))*3.6)
                #         print('yaw: ', detection_rix.yaw)
                #         print('vx:',detection_rix.velocity.x,'vy:',detection_rix.velocity.y)

                #     else:
                #         detection_rix.velocity.x = 0.1
                #         detection_rix.velocity.y = 0.1
                #         detection_rix.yaw = self.default_heading
                #     kalman_filter_list[str(obj_meta.object_id)].predict()

                # else:
                #     id_list.append(obj_meta.object_id)
                #     id_list.sort()

                #     list_temp = []
                #     list_temp.append([tp_xy[0], tp_xy[1]])
                #     trace_list[obj_meta.object_id] = list_temp
                #     # initialize a Kalman filter for traker (update by Handai)
                #     kalman_filter_list[str(obj_meta.object_id)] = KalmanTracker(np.array([detection_rix.position.x,detection_rix.position.y]).reshape(2,1))
                #     kalman_filter_list[str(obj_meta.object_id)].update(np.array([detection_rix.position.x,detection_rix.position.y]).reshape(2,1))
                #     # estimate the tracker velocity by using Kalman filter (update by Handai)
                #     if len(trace_list[obj_meta.object_id]) >=3:
                #         estimate = kalman_filter_list[str(obj_meta.object_id)].get_state()
                #         posi_x = estimate[0]
                #         posi_y = estimate[1]
                #         vel_xe = estimate[2]
                #         vel_ye = estimate[3]
                #         if math.sqrt(float(vel_xe)*float(vel_xe)+float(vel_ye)*float(vel_ye))*3.6 >= 0.5:
                #             detection_rix.velocity.x = float(vel_xe)
                #             detection_rix.velocity.y = float(vel_ye)
                #             detection_rix.position.x = float(posi_x)
                #             detection_rix.position.y = float(posi_y)
                #             # calculate heading
                #             ang = math.atan2(detection_rix.velocity.y, detection_rix.velocity.x)
                #             detection_rix.yaw = ang
                #         else:
                #             detection_rix.velocity.x = 0.01
                #             detection_rix.velocity.y = 0.01
                #             detection_rix.yaw = self.default_heading
                #     else:
                #         detection_rix.velocity.x = 0.1
                #         detection_rix.velocity.y = 0.1
                #         detection_rix.yaw = self.default_heading
                #     kalman_filter_list[str(obj_meta.object_id)].predict()
                
                # # print("111111 detection_rix class is %s" %(detection_rix.classification[-1].obj_class))
                # if  (detection_rix.classification[-1].obj_class in self.class_filter) \
                #     and \
                #     (detection_rix.classification[-1].confidence >= self.confidence_filter) \
                #     and \
                #     (detection_rix.position.x>=self.IOU_filter_x[0] and detection_rix.position.x<=self.IOU_filter_x[1]) \
                #     and \
                #     (detection_rix.position.y>=self.IOU_filter_y[0] and detection_rix.position.y<=self.IOU_filter_y[1]) \
                #     and \
                #     (Pixel_IOU_flag == True):
                #     # print(detection_rix.classification[-1].obj_class,"::::",self.class_filter) self.IOU_filter
                #     temp_data.append(detection_rix)
                #     if calibration:
                #         detection_calibration = detection_rix
                #         detection_calibration.position.x = float(pts[0][0][0])
                #         detection_calibration.position.y = float(pts[0][0][1])
                #         temp_calibration.append(detection_calibration)

                # #trace_list[result.id].append(loc)
                # #frame_number[result.id].append(self.frames)
                # if len(id_list) > 255:
                #     trace_list.clear()
                #     id_list.clear()
                #     kalman_filter_list.clear()# update by Handai
                # # Periodically check for objects with borderline confidence value that may be false positive detections.
                # # If such detections are found, annotate the frame with bboxes and confidence value.
                # # Save the annotated frame to file.
                # if((saved_count["stream_"+str(frame_meta.pad_index)]%30==0) and (obj_meta.confidence>0.3 and obj_meta.confidence<0.5)):
                #     if is_first_obj:
                #         is_first_obj = False
                #         # Getting Image data using nvbufsurface
                #         # the input should be address of buffer and batch_id
                #         n_frame=pyds.get_nvds_buf_surface(hash(gst_buffer),frame_meta.batch_id)
                #         #convert python array into numy array format.
                #         frame_image=np.array(n_frame,copy=True,order='C')
                #         #covert the array into cv2 default color format
                #         frame_image=cv2.cvtColor(frame_image,cv2.COLOR_RGBA2BGRA)

                #     # save_image = True
                #     save_image=False
                    
                #     # print("xxx_frame_image:::", frame_image)
                #     # print("yyy_obj_meta:::",obj_meta)
                #     # frame_image=self.draw_bounding_boxes(frame_image,obj_meta,obj_meta.confidence)
                #     frame_image=self.draw_bounding_boxes(frame_image,obj_meta.rect_params,obj_meta.class_id,obj_meta.confidence)
                try:
                    l_obj=l_obj.next
                except StopIteration:
                    break


            # Get frame rate through this probe
            # fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()
            #print(fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps(),'================================')
            # Publishing message with output of detection inference 
            # msg.header._frame_id = frame_id
            # msg.header.stamp = self.get_clock().now().to_msg()       
            # header = Header()
            # header.frame_id = frame_id
            # header.stamp = self.get_clock().now().to_msg() 
            # msg_rix.header = header
            # msg_rix.objects = temp_data
            # #self.publisher_detection.publish(msg)
            # self.publisher_detection_rix.publish(msg_rix)
            # if calibration:
            #     msg_calibration = Objects()
            #     msg_calibration.header = header
            #     msg_calibration.objects = temp_calibration
            #     self.publisher_detection_calibration.publish(msg_calibration)

            # if save_image:
            #     folder_name = "./"
            #     cv2.imwrite(folder_name+"/stream_"+str(frame_meta.pad_index)+"/frame_"+str(frame_number)+".jpg",frame_image)
            # saved_count["stream_"+str(frame_meta.pad_index)]+=1
            try:
                l_frame=l_frame.next
            except StopIteration:
                break

        return Gst.PadProbeReturn.OK



def cb_newpad(decodebin, decoder_src_pad,data):
    print("In cb_newpad\n")
    caps=decoder_src_pad.get_current_caps()
    if not caps:
        caps = decoder_src_pad.query_caps()
    gststruct=caps.get_structure(0)
    gstname=gststruct.get_name()
    source_bin=data
    features=caps.get_features(0)

    # Need to check if the pad created by the decodebin is for video and not
    # audio.
    print("gstname=",gstname)
    if(gstname.find("video")!=-1):
        # Link the decodebin pad only if decodebin has picked nvidia
        # decoder plugin nvdec_*. We do this by checking if the pad caps contain
        # NVMM memory features.
        print("features=",features)
        if features.contains("memory:NVMM"):
            # Get the source bin ghost pad
            bin_ghost_pad=source_bin.get_static_pad("src")
            if not bin_ghost_pad.set_target(decoder_src_pad):
                sys.stderr.write("Failed to link decoder src pad to source bin ghost pad\n")
        else:
            sys.stderr.write(" Error: Decodebin did not pick nvidia decoder plugin.\n")

def decodebin_child_added(child_proxy,Object,name,user_data):
    print("Decodebin child added:", name, "\n")
    if(name.find("decodebin") != -1):
        Object.connect("child-added",decodebin_child_added,user_data)

    if "source" in name:
        source_element = child_proxy.get_by_name("source")
        if source_element.find_property('drop-on-latency') != None:
            Object.set_property("drop-on-latency", True)



def create_source_bin(index,uri):
    print("Creating source bin")

    # Create a source GstBin to abstract this bin's content from the rest of the
    # pipeline
    bin_name="source-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    # Source element for reading from the uri.
    # We will use decodebin and let it figure out the container format of the
    # stream and the codec and plug the appropriate demux and decode plugins.
    if file_loop:
        # use nvurisrcbin to enable file-loop
        uri_decode_bin=Gst.ElementFactory.make("nvurisrcbin", "uri-decode-bin")
        uri_decode_bin.set_property("file-loop", 1)
        uri_decode_bin.set_property("cudadec-memtype", 0)
    else:
        uri_decode_bin=Gst.ElementFactory.make("uridecodebin", "uri-decode-bin")
    if not uri_decode_bin:
        sys.stderr.write(" Unable to create uri decode bin \n")
    # We set the input uri to the source element
    uri_decode_bin.set_property("uri",uri)
    # Connect to the "pad-added" signal of the decodebin which generates a
    # callback once a new pad for raw data has beed created by the decodebin
    uri_decode_bin.connect("pad-added",cb_newpad,nbin)
    uri_decode_bin.connect("child-added",decodebin_child_added,nbin)

    # We need to create a ghost pad for the source bin which will act as a proxy
    # for the video decoder src pad. The ghost pad will not have a target right
    # now. Once the decode bin creates the video decoder and generates the
    # cb_newpad callback, we will set the ghost pad target to the video decoder
    # src pad.
    Gst.Bin.add(nbin,uri_decode_bin)
    bin_pad=nbin.add_pad(Gst.GhostPad.new_no_target("src",Gst.PadDirection.SRC))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin

def build_pipeline(args, requested_pgie=None, config=None, disable_probe=False):
    global perf_data
    # print("yyyyyyyyyyyyyy:   ",args)
    perf_data = PERF_DATA(len(args))

    number_sources=len(args)

    global wxy
    wxy = args
    # print("wwww:  ", perf_data)

    # Standard GStreamer initialization
    Gst.init(None)

    # Create gstreamer elements */
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    global pipeline
    pipeline = Gst.Pipeline()
    is_live = False

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
    print("Creating streamux \n ")

    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")

    pipeline.add(streammux)
    for i in range(number_sources):
        print("Creating source_bin ",i," \n ")
        uri_name=args[i]
        if uri_name.find("rtsp://") == 0 :
            is_live = True
        source_bin=create_source_bin(i, uri_name)
        if not source_bin:
            sys.stderr.write("Unable to create source bin \n")
        pipeline.add(source_bin)
        padname="sink_%u" %i
        sinkpad= streammux.get_request_pad(padname) 
        if not sinkpad:
            sys.stderr.write("Unable to create sink pad bin \n")
        srcpad=source_bin.get_static_pad("src")
        if not srcpad:
            sys.stderr.write("Unable to create src pad bin \n")
        srcpad.link(sinkpad)
    queue1=Gst.ElementFactory.make("queue","queue1")
    queue2=Gst.ElementFactory.make("queue","queue2")
    queue3=Gst.ElementFactory.make("queue","queue3")
    queue4=Gst.ElementFactory.make("queue","queue4")
    queue5=Gst.ElementFactory.make("queue","queue5")
    pipeline.add(queue1)
    pipeline.add(queue2)
    pipeline.add(queue3)
    pipeline.add(queue4)
    pipeline.add(queue5)

    nvdslogger = None

    print("Creating Pgie \n ")
    if requested_pgie != None and (requested_pgie == 'nvinferserver' or requested_pgie == 'nvinferserver-grpc') :
        pgie = Gst.ElementFactory.make("nvinferserver", "primary-inference")
    elif requested_pgie != None and requested_pgie == 'nvinfer':
        pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    else:
        pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")

    if not pgie:
        sys.stderr.write(" Unable to create pgie :  %s\n" % requested_pgie)

    if disable_probe:
        # Use nvdslogger for perf measurement instead of probe function
        print ("Creating nvdslogger \n")
        nvdslogger = Gst.ElementFactory.make("nvdslogger", "nvdslogger")

    print("Creating tiler \n ")
    tiler=Gst.ElementFactory.make("nvmultistreamtiler", "nvtiler")
    if not tiler:
        sys.stderr.write(" Unable to create tiler \n")
    print("Creating nvvidconv \n ")
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")
    print("Creating nvosd \n ")
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")
    nvosd.set_property('process-mode',OSD_PROCESS_MODE)
    nvosd.set_property('display-text',OSD_DISPLAY_TEXT)

    if file_loop:
        if is_aarch64():
            # Set nvbuf-memory-type=4 for aarch64 for file-loop (nvurisrcbin case)
            streammux.set_property('nvbuf-memory-type', 4)
        else:
            # Set nvbuf-memory-type=2 for x86 for file-loop (nvurisrcbin case)
            streammux.set_property('nvbuf-memory-type', 2)

    if no_display:  # true why not work
        print("Creating Fakesink \n")
        sink = Gst.ElementFactory.make("fakesink", "fakesink")
        sink.set_property('enable-last-sample', 0)
        sink.set_property('sync', 0)
    else:
        if is_aarch64():
            print("Creating nv3dsink \n")
            sink = Gst.ElementFactory.make("nv3dsink", "nv3d-sink")
            if not sink:
                sys.stderr.write(" Unable to create nv3dsink \n")
        else:
            print("Creating EGLSink \n")
            # sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
            sink = Gst.ElementFactory.make("fakesink", "nvvideo-renderer")
            if not sink:
                sys.stderr.write(" Unable to create egl sink \n")

    if not sink:
        sys.stderr.write(" Unable to create sink element \n")

    if is_live:
        print("At least one of the sources is live")
        streammux.set_property('live-source', 1)

    streammux.set_property('width', 1920)
    streammux.set_property('height', 1080)
    streammux.set_property('batch-size', number_sources)
    streammux.set_property('batched-push-timeout', 4000000)
    if requested_pgie == "nvinferserver" and config != None:
        pgie.set_property('config-file-path', config)
    elif requested_pgie == "nvinferserver-grpc" and config != None:
        pgie.set_property('config-file-path', config)
    elif requested_pgie == "nvinfer" and config != None:
        pgie.set_property('config-file-path', config)
    else:
        pgie.set_property('config-file-path', "dstest3_pgie_config.txt")
    pgie_batch_size=pgie.get_property("batch-size")
    if(pgie_batch_size != number_sources):
        print("WARNING: Overriding infer-config batch-size",pgie_batch_size," with number of sources ", number_sources," \n")
        pgie.set_property("batch-size",number_sources)
    tiler_rows=int(math.sqrt(number_sources))
    tiler_columns=int(math.ceil((1.0*number_sources)/tiler_rows))
    tiler.set_property("rows",tiler_rows)
    tiler.set_property("columns",tiler_columns)
    tiler.set_property("width", TILED_OUTPUT_WIDTH)
    tiler.set_property("height", TILED_OUTPUT_HEIGHT)
    sink.set_property("qos",0)

    print("Adding elements to Pipeline \n")
    pipeline.add(pgie)
    if nvdslogger:
        pipeline.add(nvdslogger)
    pipeline.add(tiler)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(sink)

    print("Linking elements in the Pipeline \n")
    streammux.link(queue1)
    queue1.link(pgie)
    pgie.link(queue2)
    if nvdslogger:
        queue2.link(nvdslogger)
        nvdslogger.link(tiler)
    else:
        queue2.link(tiler)
    tiler.link(queue3)
    queue3.link(nvvidconv)
    nvvidconv.link(queue4)
    queue4.link(nvosd)
    nvosd.link(queue5)
    queue5.link(sink)   

    # create an event loop and feed gstreamer bus mesages to it
    global loop
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)
    pgie_src_pad=pgie.get_static_pad("src")
    if not pgie_src_pad:
        sys.stderr.write(" Unable to get src pad \n")
    else:
        if  not disable_probe:
            # pgie_src_pad.add_probe(Gst.PadProbeType.BUFFER, tiler_sink_pad_buffer_probe, 0)
            pgie_src_pad.add_probe(Gst.PadProbeType.BUFFER, pgie_src_pad_buffer_probe, 0)
            # perf callback function to print fps every 5 sec
            GLib.timeout_add(5000, perf_data.perf_print_callback)

    # pipeline_run()

def pipeline_run():
    # List the sources
    print("Now playing...")
    for i, source in enumerate(wxy):
        print(i, ": ", source)
    print("Starting pipeline \n")
    # start play back and listed to events		
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup
    print("Exiting app\n")
    pipeline.set_state(Gst.State.NULL)

def parse_args():

    parser = argparse.ArgumentParser(prog="deepstream_test_3",
                    description="deepstream-test3 multi stream, multi model inference reference app")
    parser.add_argument(
        "-i",
        "--input",
        help="Path to input streams",
        nargs="+",
        metavar="URIs",
        default=["a"],
        required=True,
    )
    parser.add_argument(
        "-c",
        "--configfile",
        metavar="config_location.txt",
        default=None,
        help="Choose the config-file to be used with specified pgie",
    )
    parser.add_argument(
        "-g",
        "--pgie",
        default=None,
        help="Choose Primary GPU Inference Engine",
        choices=["nvinfer", "nvinferserver", "nvinferserver-grpc"],
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        default=False,
        dest='no_display',
        help="Disable display of video output",
    )
    parser.add_argument(
        "--file-loop",
        action="store_true",
        default=False,
        dest='file_loop',
        help="Loop the input file sources after EOS",
    )
    parser.add_argument(
        "--disable-probe",
        action="store_true",
        default=False,
        dest='disable_probe',
        help="Disable the probe function and use nvdslogger for FPS",
    )
    parser.add_argument(
        "-s",
        "--silent",
        action="store_true",
        default=False,
        dest='silent',
        help="Disable verbose output",
    )
    # Check input arguments
    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)
    args = parser.parse_args()

    # print("wwwwwww:   ",args)

    stream_paths = args.input
    pgie = args.pgie
    config = args.configfile
    disable_probe = args.disable_probe
    global no_display
    global silent
    global file_loop
    no_display = args.no_display
    silent = args.silent
    file_loop = args.file_loop

    if config and not pgie or pgie and not config:
        sys.stderr.write ("\nEither pgie or configfile is missing. Please specify both! Exiting...\n\n\n\n")
        parser.print_help()
        sys.exit(1)
    if config:
        config_path = Path(config)
        if not config_path.is_file():
            sys.stderr.write ("Specified config-file: %s doesn't exist. Exiting...\n\n" % config)
            sys.exit(1)

    print(vars(args))
    # print("xxxxxxxxxxx:   ",args)
    return stream_paths, pgie, config, disable_probe

if __name__ == '__main__':
    stream_paths, pgie, config, disable_probe = parse_args()
    build_pipeline(stream_paths, pgie, config, disable_probe)
    pipeline_run()
