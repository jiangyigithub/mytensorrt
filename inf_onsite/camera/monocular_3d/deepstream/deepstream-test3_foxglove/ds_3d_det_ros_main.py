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
import numpy as np

import rclpy
from rclpy.node import Node
from perception_kit_msgs.msg import Object
from perception_kit_msgs.msg import Objects
from perception_kit_msgs.msg import Classification
from std_msgs.msg import Header
from lib.helpers.depth_geometry import projection, kdtree

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

# from vision_msgs.msg import Classification2D, ObjectHypothesis, ObjectHypothesisWithPose, BoundingBox2D, Detection2D, Detection2DArray


import pyds

import os
os.environ["GST_DEBUG_DUMP_DOT_DIR"] = "/home/icv/neil_dev/deepstream/deepstream-test3/debug"
os.putenv('GST_DEBUG_DUMP_DIR_DIR', '/home/icv/neil_dev/deepstream/deepstream-test3/debug')

depth_geometry_open = False
e_roi_open = False
s_roi_open = False
w_roi_open = True
n_roi_open = False

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
class MC3dObjDet_Publisher(Node):

    def img_to_rect( self, u, v, depth_rect):
        """
        :param u: (N)
        :param v: (N)
        :param depth_rect: (N)
        :return:
        """
        # x = ((u - 979.18207798) * depth_rect) / 997.30810007 
        # y = ((v - 517.0099155) * depth_rect) / 997.04838962 
        x = ((u - 1143.7368) * depth_rect) / 1367.8905 
        y = ((v - 722.559216) * depth_rect) / 1373.59794 
        pts_rect = np.concatenate(
            ((np.array)(x).reshape(-1, 1), (np.array)(y).reshape(-1, 1), (np.array)(depth_rect).reshape(-1, 1)),
            axis=1)
        return pts_rect

    def roi_filter_img_down(self,u,v,a,b):
        roi_v = a*u+b
        if v > roi_v:
            return True
        else:
            return False

    # tiler_sink_pad_buffer_probe  will extract metadata received on tiler src pad
    # and update params for drawing rectangle, object information etc.
    def tiler_sink_pad_buffer_probe(self, pad, info, u_data):
        frame_number = 0
        num_rects = 0
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

            # Getting Image data using nvbufsurface
            # the input should be address of buffer and batch_id
            n_frame = pyds.get_nvds_buf_surface(hash(gst_buffer), frame_meta.batch_id)
            # n_frame = draw_bounding_boxes(n_frame, obj_meta, obj_meta.confidence)


            # convert python array into numpy array format in the copy mode.
            frame_copy = np.array(n_frame, copy=True, order='C')
            # convert the array into cv2 default color format
            frame_copy = cv2.cvtColor(frame_copy, cv2.COLOR_RGBA2BGRA)
            # if is_aarch64(): # If Jetson, since the buffer is mapped to CPU for retrieval, it must also be unmapped 
            #     pyds.unmap_nvds_buf_surface(hash(gst_buffer), frame_meta.batch_id) # The unmap call should be made after operations with the original array are complete.
            #                                                                         #  The original array cannot be accessed after this call.
            # print('-------Generate compressed image------')
            # print(len(n_frame), ",", len(frame_copy))

            # img_path = "frame_{}.png".format(frame_number)
            # cv2.imwrite(img_path, frame_copy)

            cp_image = self.bridge.cv2_to_compressed_imgmsg(frame_copy)
            cp_image.header.stamp = self.get_clock().now().to_msg()
            cp_image.header.frame_id = "layered_map_enu"

            self.publisher_cp_image.publish(cp_image)

            try:
                l_frame = l_frame.next
            except StopIteration:
                break

        return Gst.PadProbeReturn.OK


    def pgie_src_pad_buffer_probe(self,pad,info,u_data):
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
            objs_3d_det = Objects()
            temp_data = []
            print('---------------------------objs_detected---------------------------')
            while l_obj is not None:
                try: 
                    # Casting l_obj.data to pyds.NvDsObjectMeta
                    obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
                except StopIteration:
                    break
                obj_det = Object()
                # print("class_id:   ",(obj_meta.class_id))
                # print("confidence: ",'%.2f'%(obj_meta.confidence))
                # print("left:       ",'%.2f'%(obj_meta.rect_params.left))
                # print("top:        ",'%.4f'%(obj_meta.rect_params.top))
                # print("width:      ",'%.2f'%(obj_meta.rect_params.width))
                # print("height:     ",'%.6f'%(obj_meta.rect_params.height))
                obj_det.header.stamp = self.get_clock().now().to_msg() 
                obj_det.header.frame_id = "layered_map_enu"
                # obj_det.header.frame_id = ""
                # obj_det.id = obj_meta.class_id
                classification = Classification()
                cl = []
                classification.obj_class = "car" ##obj_meta.class_id
                classification.confidence =  0.8 
                cl.append(classification)
                obj_det.classification = cl
                obj_det.id = obj_meta.class_id ##need tracker process 
                obj_det.width = (obj_meta.rect_params.left%1E4)/1E2
                obj_det.length = int(obj_meta.confidence/1E4)/1E2
                obj_det.height = int(obj_meta.rect_params.left/1E4)/1E2
                # obj_det.yaw = obj_meta.rect_params.height+math.pi/2-0.029865
                obj_det.yaw = obj_meta.rect_params.height # -math.pi/2+0.05
                obj_det.yaw = -obj_det.yaw+math.pi/2+0.05 # hongye west
                # obj_det.yaw = -obj_det.yaw+0.05 # hongye south
                # obj_det.yaw = -obj_det.yaw+0.03 # hongye north
                obj_det.existence_probability = (obj_meta.confidence%1E4)/1E3
                temp_x3d = int(obj_meta.rect_params.width/1E4)
                temp_y3d = int(obj_meta.rect_params.width%1E4)
                ## change for oritention
                
                tan_k = (temp_x3d-1920/2)/(1080-temp_y3d)
                # print("yaw_bef:  ", obj_det.yaw)
                if(tan_k <0):
                    obj_det.yaw = tan_k*0.7+obj_det.yaw
                    # print("yaw_aft:  ", obj_det.yaw)
                ##
                # print("--------------------",temp_x3d,"   ",temp_y3d)
                depth = obj_meta.rect_params.top
                depth = depth/1.4 ## w
                # depth = depth/1.55 ## n
                # print(obj_det.yaw,"----------",obj_meta.rect_params.height)
                # print("--------before------",depth)
                if depth_geometry_open :
                    # depth = projection.find_point(temp_x3d,temp_y3d,obj_det.height/2-0.4)
                    depth = projection.find_point(temp_x3d,temp_y3d,obj_det.height/2)
                    # print("-------after-------",depth)
                # locations = self.img_to_rect(temp_x3d, temp_y3d, depth) #camera coordinate
                locations = projection.reverse_porjection(temp_x3d, temp_y3d, depth)
                locations = (locations).reshape(-1)

                obj_det.position.x = float(locations[0])
                obj_det.position.y = float(locations[1])

                # print("left--------------:",obj_meta.rect_params.left)
                # print("width--------------:",obj_meta.rect_params.width)
                # print("y----------:",obj_det.position.y)

                # debug info
                # this data range is error beside yaw & yaw rate
                obj_det.width_variance = obj_meta.confidence
                obj_det.length_variance = obj_meta.rect_params.left
                obj_det.height_variance = obj_meta.rect_params.width
                obj_det.yaw_rate = obj_meta.rect_params.height
                obj_det.x_offset = obj_meta.rect_params.top
                # print(obj_det.x_offset)
                # print("left_after--------------:",obj_det.length_variance)
                # print("width_left--------------:",obj_det.height_variance)
                # temp_data.append(obj_det)

                if (w_roi_open):
                    temp_l12 = self.roi_filter_img_down(temp_x3d,temp_y3d,-0.689,770)
                    temp_l34 = self.roi_filter_img_down(temp_x3d,temp_y3d,0.348,-80)
                    if temp_y3d > 350:
                        temp_l23 = True
                    else:
                        temp_l23 = False
                    if(temp_l12 and temp_l34 and temp_l23):
                        print("uv:    ",temp_x3d, "    ", temp_y3d)
                        temp_data.append(obj_det)
                elif (n_roi_open):
                    temp_l12 = self.roi_filter_img_down(temp_x3d,temp_y3d,-0.65,650)
                    temp_l56 = self.roi_filter_img_down(temp_x3d,temp_y3d,0.528,-558)
                    if temp_y3d > 150:
                        temp_l45 = True
                    else:
                        temp_l45 = False
                    if temp_y3d < 980:
                        temp_l_D = True
                    else:
                        temp_l_D = False
                    if temp_x3d > 200:
                        temp_l_L = True
                    else:
                        temp_l_L = False
                    if temp_x3d < 1820:
                        temp_l_R = True
                    else:
                        temp_l_R = False
                    if(temp_l12 and temp_l45 and temp_l56 and temp_l_D and temp_l_L and temp_l_R):
                        print("uv:    ",temp_x3d, "    ", temp_y3d)
                        temp_data.append(obj_det)
                else:
                    temp_data.append(obj_det)
                try: 
                    l_obj=l_obj.next
                except StopIteration:
                    break

            header = Header()
            header.stamp = self.get_clock().now().to_msg() 
            objs_3d_det.header = header
            objs_3d_det.header.frame_id = "layered_map_enu"
            objs_3d_det.objects = temp_data   
            print(len(temp_data))
            self.publisher_3d_obj_det.publish(objs_3d_det)
            # #Update frame rate through this probe
            stream_index = "stream{0}".format(frame_meta.pad_index)
            global perf_data
            perf_data.update_fps(stream_index)
            try:
                l_frame=l_frame.next
            except StopIteration:
                break

        return Gst.PadProbeReturn.OK
    
    def cb_newpad(self, decodebin, decoder_src_pad,data):
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

    def decodebin_child_added(self, child_proxy,Object,name,user_data):
        print("Decodebin child added:", name, "\n")
        if(name.find("decodebin") != -1):
            Object.connect("child-added",self.decodebin_child_added,user_data)
        
        print("------------ Prepare to Set skip_frames for nvv4l2decoder \n")
        print(name)
        if(name.find("nvv4l2decoder0") != -1):
            print("Set skip_frames for nvv4l2decoder \n")
            #Use CUDA unified memory in the pipeline so frames
            # can be easily accessed on CPU in Python.
            Object.set_property("cudadec-memtype", 2)
            # Object.set_property("skip_frames",2)
            # Object.set_property("drop-frame-interval",3)
            # Object.set_property("low-latency_mode",True)
        print("------------ Finish to Set skip_frames for nvv4l2decoder \n")
        if "source" in name:
            source_element = child_proxy.get_by_name("source")
            if source_element.find_property('drop-on-latency') != None:
                Object.set_property("drop-on-latency", True)

    def create_source_bin(self, index,uri):
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
        uri_decode_bin.connect("pad-added",self.cb_newpad,nbin)
        uri_decode_bin.connect("child-added",self.decodebin_child_added,nbin)

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

    def __init__(self):
        super().__init__('MC3dObjDet_Publisher')
        self.parse_args()
        global perf_data
        # print("ooooo",self.args)
        # self.args = list(['rtsp://service:Icv%24%241234@192.168.23.104'])
        # self.args = list(['rtsp://service:Icv%24%241234@192.168.7.72'])
        # self.args = list(['rtsp://service:Icv%24%241234@192.168.7.74'])
        # self.args = list(['rtsp://service:Icv%24%241234@192.168.7.73'])
        self.args = list(['file://////home/icv-ai/workspaces/deepstream/samples/streams/sample_1080p_h264.mp4']) 

        perf_data = PERF_DATA(len(self.args))
        number_sources=len(self.args)

        # the 'CVBridge' is a python_class, must have a instance.
        # That means "cv2_to_imgmsg() and cv2_to_compressed_imgmsg must be called with CvBridge instance"

        self.bridge = CvBridge()
        self.publisher_3d_obj_det = self.create_publisher(Objects, "mc_obj_det", 1000)
        self.publisher_cp_image = self.create_publisher(CompressedImage, "icv_cp_image", 100)

        # Standard GStreamer initialization
        Gst.init(None)

        # Create gstreamer elements */
        # Create Pipeline element that will form a connection of other elements
        print("Creating Pipeline \n ")
        self.pipeline = Gst.Pipeline()
        is_live = False

        if not self.pipeline:
            sys.stderr.write(" Unable to create Pipeline \n")
        print("Creating streamux \n ")

        # Create nvstreammux instance to form batches from one or more sources.
        streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
        if not streammux:
            sys.stderr.write(" Unable to create NvStreamMux \n")

        self.pipeline.add(streammux)
        for i in range(number_sources):
            print("Creating source_bin ",i," \n ")
            uri_name=self.args[i]
            if uri_name.find("rtsp://") == 0 :
                is_live = True
            source_bin=self.create_source_bin(i, uri_name)
            if not source_bin:
                sys.stderr.write("Unable to create source bin \n")
            self.pipeline.add(source_bin)
            padname="sink_%u" %i
            sinkpad= streammux.get_request_pad(padname) 
            if not sinkpad:
                sys.stderr.write("Unable to create sink pad bin \n")
            srcpad=source_bin.get_static_pad("src")
            if not srcpad:
                sys.stderr.write("Unable to create src pad bin \n")
            srcpad.link(sinkpad)
        

        print("Creating Pgie \n ")
        pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
        if not pgie:
            sys.stderr.write(" Unable to create pgie \n")
        # Add nvvidconv1 and filter1 to convert the frames to RGBA
        # which is easier to work with in Python.
        print("Creating nvvidconv1 \n ")
        nvvidconv1 = Gst.ElementFactory.make("nvvideoconvert", "convertor1")
        if not nvvidconv1:
            sys.stderr.write(" Unable to create nvvidconv1 \n")
        print("Creating filter1 \n ")
        caps1 = Gst.Caps.from_string("video/x-raw(memory:NVMM), format=RGBA")
        filter1 = Gst.ElementFactory.make("capsfilter", "filter1")
        if not filter1:
            sys.stderr.write(" Unable to get the caps filter1 \n")
        filter1.set_property("caps", caps1)
        print("Creating tiler \n ")
        tiler = Gst.ElementFactory.make("nvmultistreamtiler", "nvtiler")
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
        if is_aarch64():
            print("Creating nv3dsink \n")
            sink = Gst.ElementFactory.make("nv3dsink", "nv3d-sink")
            if not sink:
                sys.stderr.write(" Unable to create nv3dsink \n")
        else:
            print("Creating EGLSink \n")
            sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
            if not sink:
                sys.stderr.write(" Unable to create egl sink \n")

        if is_live:
            print("Atleast one of the sources is live")
            streammux.set_property('live-source', 1)

        streammux.set_property('width', 1920)
        streammux.set_property('height', 1080)
        streammux.set_property('batch-size', number_sources)
        streammux.set_property('batched-push-timeout', 4000000)
        pgie.set_property('config-file-path', "dstest3_pgie_config.txt")
        pgie_batch_size = pgie.get_property("batch-size")
        if (pgie_batch_size != number_sources):
            print("WARNING: Overriding infer-config batch-size", pgie_batch_size, " with number of sources ",
                number_sources, " \n")
            pgie.set_property("batch-size", number_sources)
        tiler_rows = int(math.sqrt(number_sources))
        tiler_columns = int(math.ceil((1.0 * number_sources) / tiler_rows))
        tiler.set_property("rows", tiler_rows)
        tiler.set_property("columns", tiler_columns)
        tiler.set_property("width", TILED_OUTPUT_WIDTH)
        tiler.set_property("height", TILED_OUTPUT_HEIGHT)

        sink.set_property("sync", 0)
        sink.set_property("qos", 0)

        if not is_aarch64():
            # Use CUDA unified memory in the pipeline so frames
            # can be easily accessed on CPU in Python.
            # mem_type = int(pyds.NVBUF_MEM_CUDA_UNIFIED)
            mem_type = 3
            streammux.set_property("nvbuf-memory-type", mem_type)
            nvvidconv.set_property("nvbuf-memory-type", mem_type)
            nvvidconv1.set_property("nvbuf-memory-type", mem_type)
            tiler.set_property("nvbuf-memory-type", mem_type)

        print("Adding elements to Pipeline \n")
        self.pipeline.add(pgie)
        self.pipeline.add(tiler)
        self.pipeline.add(nvvidconv)
        self.pipeline.add(filter1)
        self.pipeline.add(nvvidconv1)
        self.pipeline.add(nvosd)
        self.pipeline.add(sink)

        print("Linking elements in the Pipeline \n")
        streammux.link(pgie)
        pgie.link(nvvidconv1)
        nvvidconv1.link(filter1)
        filter1.link(tiler)
        tiler.link(nvvidconv)
        nvvidconv.link(nvosd)
        nvosd.link(sink)

        # create an event loop and feed gstreamer bus mesages to it
        # global loop
        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect ("message", bus_call, self.loop)


        pgie_src_pad=pgie.get_static_pad("src")
        if not pgie_src_pad:
            sys.stderr.write(" Unable to get src pad \n")
        else:
            pgie_src_pad.add_probe(Gst.PadProbeType.BUFFER, self.pgie_src_pad_buffer_probe, 0)
            Gst.debug_bin_to_dot_file(self.pipeline, Gst.DebugGraphDetails.ALL, "pipeline_wx")
            # perf callback function to print fps every 5 sec
            GLib.timeout_add(5000, perf_data.perf_print_callback)

        # tiler_sink_pad = tiler.get_static_pad("sink")
        tiler_sink_pad = tiler.get_static_pad("src")
        if not tiler_sink_pad:
            sys.stderr.write(" Unable to get src pad \n")
        else:
            tiler_sink_pad.add_probe(Gst.PadProbeType.BUFFER, self.tiler_sink_pad_buffer_probe, 0)
            # perf callback function to print fps every 5 sec
            GLib.timeout_add(5000, perf_data.perf_print_callback)

    def pipeline_run(self):
        # List the sources
        print("Now playing...")
        for i, source in enumerate(self.args):
            print(i, ": ", source)
        print("Starting pipeline \n")
        # start play back and listed to events		
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
        except:
            pass
        # cleanup
        print("Exiting app\n")
        self.pipeline.set_state(Gst.State.NULL)

    def parse_args(self):

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
        self.args = parser.parse_args()

        self.stream_paths = self.args.input
        self.pgie = self.args.pgie
        self.config = self.args.configfile
        self.disable_probe = self.args.disable_probe
        global no_display
        global silent
        global file_loop
        no_display = self.args.no_display
        silent = self.args.silent
        file_loop = self.args.file_loop

        if self.config and not self.pgie or self.pgie and not self.config:
            sys.stderr.write ("\nEither pgie or configfile is missing. Please specify both! Exiting...\n\n\n\n")
            parser.print_help()
            sys.exit(1)
        if self.config:
            config_path = Path(self.config)
            if not config_path.is_file():
                sys.stderr.write ("Specified config-file: %s doesn't exist. Exiting...\n\n" % self.config)
                sys.exit(1)

        print(vars(self.args))
        # return stream_paths, pgie, config, disable_probe

if __name__ == '__main__':

    rclpy.init()
    # camera_pose_info =[[-1.740657, 0.030855, 0.029865],[20.169886, -14.505331, 6.657059]] # bt lens
    # camera_pose_info =[[-1.873035, 0.016564, -0.011060],[19.743244, -16.696733, 6.745940]] # new lens
    # camera_pose_info =[[-1.816646, -0.032217, 0.830739],[38.567593, -9.891174, 7.325952]] # new lens s
    camera_pose_info =[[-1.771768, -0.048427, -0.807171],[-11.039729, -1.951577, 7.277434]] # new lens w
    # camera_pose_info =[[-1.915713, -0.012382, -2.266973],[-2.242676, 41.932842, 7.313895]] # new lens n
    projection.projection_init(camera_pose_info)
    mc3dobjdet_publisher = MC3dObjDet_Publisher()
    # stream_paths, pgie, config, disable_probe = mc3dobjdet_publisher.parse_args()    
    # mc3dobjdet_publisher.build_pipeline(stream_paths, pgie, config, disable_probe)
    mc3dobjdet_publisher.pipeline_run()

    mc3dobjdet_publisher.destroy_node()
    rclpy.shutdown()
