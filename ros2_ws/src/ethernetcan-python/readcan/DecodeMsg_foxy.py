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
#   IAD - Infrastructure-based Autonomous Driving (CR/RIX)
# =============================================================================
#
#
#  Created on: March 23, 2021
#      Author: zns5sgh
#     Contact: fixed-term.Simon.Zhang@cn.bosch.com
#

import rclpy
from rclpy.node import Node 
from rclpy.exceptions import ROSInterruptException

import cantools
#from numba.typed import List
import struct
import binascii
#import rospy

from std_msgs.msg import Header
# from readcan_msgs.msg import *
from readcan_msgs.msg import Canfd
from radar_gen5_msgs.msg import *

import math
import std_msgs.msg
import time

from . import point_cloud2 as pcl12
from sensor_msgs.msg import PointCloud2, PointField
import os

global db

global loca_interf
loca_interf = LocationInterface()
loca_interf.header = Header()
loca_interf.radar_ecu_timestamp = 0.0
loca_interf.raw_locations = list()
loca_interf.sensing_state = SensingState()

global blockCnt
blockCnt = -1
global ProtLocationCount
ProtLocationCount = 2



def hex_str_to_int(s):
    length = len(s)
    if length == 1:
        if '0'<= s <='9':
            s_int=int(s)
        else:
            s_int=9 + (ord(s)-ord('a'))
    return s_int


class readcan_node(Node):

    def __init__(self):
        super().__init__('readcan')
        
        # declare parameter
        self.declare_parameter('dbc_path')
        self.declare_parameter('frame_id')
        self.frame_id = self.get_parameter('frame_id').value
    
        # retrieve parameter
        self.dbc_path = self.get_parameter('dbc_path').value
        global db
        db = cantools.database.load_file(self.dbc_path)

        # create sub
        self.sub = self.create_subscription(Canfd, "tcp/canfd", self.callback, 1000)

        # create pub 
        self.pub_1 = self.create_publisher(LocationInterface, '/LocationInterface', 1000)
        self.pub_2 = self.create_publisher(PointCloud2, '/filtered_clound', 1000)
        self.pub_3 = self.create_publisher(PointCloud2, '/all_cloud', 1000)
    def callback(self, data):
        global db
        
        if (400 <= data.id <= 484 or data.id == 384):
            message_result = db.decode_message(data.id, data.data)
            self.fillin(data.id, message_result)

        if (257 <= data.id <= 356 or data.id == 256):
            message_result = db.decode_message(data.id, data.data)
            print('--------current id is'+hex(data.id))
            
            self.fillin(data.id, message_result)
    
    def fillin(self, id_int, msg):
        global blockCnt
        global loca_interf
        global ProtLocationCount

        if (257 <= id_int <= 356 or id_int == 256):
            
            if id_int == 256 and loca_interf.header.frame_id == self.frame_id:
                try:
                    # print(loca_interf.raw_locations[1])
                    self.pub_1.publish(loca_interf)
                    print("publish locationinterface")
                    self.loca_transfer(loca_interf)

                    print("256 with 257-356. publish out !!")
                    print('-------------------')
                    loca_interf = LocationInterface()
                    loca_interf.header = Header()
                    # print(loca_interf.header)
                    loca_interf.radar_ecu_timestamp = 0.0
                    loca_interf.raw_locations = list()
                    loca_interf.sensing_state = SensingState()
                    loca_interf.header.stamp = self.get_clock().now().to_msg()
                    loca_interf.header.frame_id = self.frame_id # without this /all_cloud and /location_interface is 7hz
                    blockCnt = msg['LocHeader_Blockcnt']
                    ProtLocationCount = msg['LocHeader_NumLoc']
                    #print('for raw '+ 'blockcnt' +str(blockCnt)+'--------current id is'+str(id_int))
                    #print(msg['RFC_Hdr00_ProtLocationCount'])
                    pass
                except ROSInterruptException:
                    pass
        
            elif id_int == 256 and loca_interf.header.frame_id != self.frame_id:
                loca_interf.header.stamp = self.get_clock().now().to_msg()
                loca_interf.header.frame_id = self.frame_id
                blockCnt = msg['LocHeader_Blockcnt']
                ProtLocationCount = msg['LocHeader_NumLoc']
                #print(msg['RFC_Hdr00_ProtLocationCount'])
                print ("new 256. No 257-356 !!")
                #print('for raw '+ 'blockcnt' +str(blockCnt)+'--------current id is'+str(id_int))
                print('-------------------')
            elif 257 <= id_int <= (257 + int(ProtLocationCount/40)*6 + int(ProtLocationCount/4)):
                print('for count '+str(ProtLocationCount)+'--------current id is'+str(id_int))
                
                A_str = hex(id_int-256)
                A_str = A_str[2:]
                if len(A_str)==1:
                    A_str = '0'+A_str
                elif len(A_str) == 2:
                    A_str = A_str
            
                print('for raw '+ str(msg['Loc_'+A_str+'_Blockcnt']) + 'blockcnt' +str(blockCnt)+'--------current id is'+str(id_int))

                if msg['Loc_'+A_str+'_Blockcnt'] == blockCnt:
                #if msg['Loc_'+A_str+'_Blockcnt'] != blockCnt:
                    radar_location_tmp_A = Location()
                    radar_location_tmp_B = Location()
                    radar_location_tmp_C = Location()
                    radar_location_tmp_D = Location()

                    radar_location_tmp_A.radial_distance = msg['Loc_RadialDist_'+A_str+'_001']
                    radar_location_tmp_A.radial_distance_variance = 0.0
                    radar_location_tmp_A.radial_distance_spread = 0.0
                    radar_location_tmp_A.radial_velocity = msg['Loc_RadialVelo_'+A_str+'_001']
                    radar_location_tmp_A.radial_velocity_variance = 0.0
                    radar_location_tmp_A.radial_velocity_spread = 0.0
                    radar_location_tmp_A.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_A.radial_distance_velocity_quality = msg['Loc_RadialDistVeloQly_'+A_str+'_001'] * 0.0039
                    radar_location_tmp_A.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_A.elevation_angle = msg['Loc_ElevAngle_'+A_str+'_001'] #lnl
                    radar_location_tmp_A.elevation_angle_quality = msg['Loc_ElevAngleQly_'+A_str+'_001'] * 0.0039
                    radar_location_tmp_A.elevation_angle_variance = 0.0
                    radar_location_tmp_A.azimuth_angle = msg['Loc_AziAngle_'+A_str+'_001'] #lnl
                    radar_location_tmp_A.azimuth_angle_quality = msg['Loc_AziAngleQly_'+A_str+'_001'] * 0.0039
                    radar_location_tmp_A.azimuth_angle_variance = 0.0
                    radar_location_tmp_A.rcs = msg['Loc_RCS_'+A_str+'_001']
                    radar_location_tmp_A.rssi = msg['Loc_SNR_'+A_str+'_001'] * 4.0 - 16.0
                    print('for raw '+ str(msg['Loc_Validity_'+A_str+'_001']) + 'blockcnt' +str(blockCnt)+'--------current id is'+str(id_int))
                    radar_location_tmp_A.measurement_status = 1 #msg['Loc_Validity_'+A_str+'_001'] #csa

                    radar_location_tmp_B.radial_distance = msg['Loc_RadialDist_'+A_str+'_002']
                    radar_location_tmp_B.radial_distance_variance = 0.0
                    radar_location_tmp_B.radial_distance_spread = 0.0
                    radar_location_tmp_B.radial_velocity = msg['Loc_RadialVelo_'+A_str+'_002']
                    radar_location_tmp_B.radial_velocity_variance = 0.0
                    radar_location_tmp_B.radial_velocity_spread = 0.0
                    radar_location_tmp_B.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_B.radial_distance_velocity_quality = msg['Loc_RadialDistVeloQly_'+A_str+'_002'] * 0.0039
                    radar_location_tmp_B.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_B.elevation_angle = msg['Loc_ElevAngle_'+A_str+'_002'] #lnl
                    radar_location_tmp_B.elevation_angle_quality = msg['Loc_ElevAngleQly_'+A_str+'_002']*0.0039
                    radar_location_tmp_B.elevation_angle_variance = 0.0
                    radar_location_tmp_B.azimuth_angle = msg['Loc_AziAngle_'+A_str+'_002'] #lnl
                    radar_location_tmp_B.azimuth_angle_quality = msg['Loc_AziAngleQly_'+A_str+'_002']*0.0039
                    radar_location_tmp_B.azimuth_angle_variance = 0.0
                    radar_location_tmp_B.rcs = msg['Loc_RCS_'+A_str+'_002']
                    radar_location_tmp_B.rssi = msg['Loc_SNR_'+A_str+'_002'] * 4.0 - 16.0
                    radar_location_tmp_B.measurement_status = 1 #msg['Loc_Validity_'+A_str+'_002'] #csa

                    radar_location_tmp_C.radial_distance = msg['Loc_RadialDist_'+A_str+'_003']
                    radar_location_tmp_C.radial_distance_variance = 0.0
                    radar_location_tmp_C.radial_distance_spread = 0.0
                    radar_location_tmp_C.radial_velocity = msg['Loc_RadialVelo_'+A_str+'_003']
                    radar_location_tmp_C.radial_velocity_variance = 0.0
                    radar_location_tmp_C.radial_velocity_spread = 0.0
                    radar_location_tmp_C.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_C.radial_distance_velocity_quality = msg['Loc_RadialDistVeloQly_'+A_str+'_003'] * 0.0039
                    radar_location_tmp_C.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_C.elevation_angle = msg['Loc_ElevAngle_'+A_str+'_003'] #lnl
                    radar_location_tmp_C.elevation_angle_quality = msg['Loc_ElevAngleQly_'+A_str+'_003']*0.0039
                    radar_location_tmp_C.elevation_angle_variance = 0.0
                    radar_location_tmp_C.azimuth_angle = msg['Loc_AziAngle_'+A_str+'_003'] #lnl
                    radar_location_tmp_C.azimuth_angle_quality = msg['Loc_AziAngleQly_'+A_str+'_003']*0.0039
                    radar_location_tmp_C.azimuth_angle_variance = 0.0
                    radar_location_tmp_C.rcs = msg['Loc_RCS_'+A_str+'_003']
                    radar_location_tmp_C.rssi = msg['Loc_SNR_'+A_str+'_003'] * 4.0 - 16.0
                    radar_location_tmp_C.measurement_status = 1 #msg['Loc_Validity_'+A_str+'_003'] #csa

                    radar_location_tmp_D.radial_distance = msg['Loc_RadialDist_'+A_str+'_004']
                    radar_location_tmp_D.radial_distance_variance = 0.0
                    radar_location_tmp_D.radial_distance_spread = 0.0
                    radar_location_tmp_D.radial_velocity = msg['Loc_RadialVelo_'+A_str+'_004']
                    radar_location_tmp_D.radial_velocity_variance = 0.0
                    radar_location_tmp_D.radial_velocity_spread = 0.0
                    radar_location_tmp_D.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_D.radial_distance_velocity_quality = msg['Loc_RadialDistVeloQly_'+A_str+'_004'] * 0.0039
                    radar_location_tmp_D.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_D.elevation_angle = msg['Loc_ElevAngle_'+A_str+'_004'] #lnl
                    radar_location_tmp_D.elevation_angle_quality = msg['Loc_ElevAngleQly_'+A_str+'_004']*0.0039
                    radar_location_tmp_D.elevation_angle_variance = 0.0
                    radar_location_tmp_D.azimuth_angle = msg['Loc_AziAngle_'+A_str+'_004'] #lnl
                    radar_location_tmp_D.azimuth_angle_quality = msg['Loc_AziAngleQly_'+A_str+'_004']*0.0039
                    radar_location_tmp_D.azimuth_angle_variance = 0.0
                    radar_location_tmp_D.rcs = msg['Loc_RCS_'+A_str+'_004']
                    radar_location_tmp_D.rssi = msg['Loc_SNR_'+A_str+'_004'] * 4.0 - 16.0
                    radar_location_tmp_D.measurement_status = 1 #msg['Loc_Validity_'+A_str+'_004'] #csa



                    loca_interf.raw_locations.append(radar_location_tmp_A)
                    loca_interf.raw_locations.append(radar_location_tmp_B)
                    loca_interf.raw_locations.append(radar_location_tmp_C)
                    loca_interf.raw_locations.append(radar_location_tmp_D)
                    

        if (400 <= id_int <= 484 or id_int == 384):
            
            if id_int == 384 and loca_interf.header.frame_id == self.frame_id:
                try:
                    # print(loca_interf.raw_locations[1])
                    self.pub_1.publish(loca_interf)
                    print("publish locationinterface")
                    self.loca_transfer(loca_interf)

                    print("384 with 400s. publish out !!")
                    print('-------------------')
                    loca_interf = LocationInterface()
                    loca_interf.header = Header()
                    # print(loca_interf.header)
                    loca_interf.radar_ecu_timestamp = 0.0
                    loca_interf.raw_locations = list()
                    loca_interf.sensing_state = SensingState()
                    loca_interf.header.stamp = self.get_clock().now().to_msg()
                    loca_interf.header.frame_id = self.frame_id # without this /all_cloud and /location_interface is 7hz
                    blockCnt = msg['RFC_Hdr00_ProtBlockCtr']
                    ProtLocationCount = msg['RFC_Hdr00_ProtLocationCount']
                    #print(msg['RFC_Hdr00_ProtLocationCount'])
                    pass
                except ROSInterruptException:
                    pass
        
            elif id_int == 384 and loca_interf.header.frame_id != self.frame_id:
                loca_interf.header.stamp = self.get_clock().now().to_msg()
                loca_interf.header.frame_id = self.frame_id
                blockCnt = msg['RFC_Hdr00_ProtBlockCtr']
                ProtLocationCount = msg['RFC_Hdr00_ProtLocationCount']
                #print(msg['RFC_Hdr00_ProtLocationCount'])
                print ("new 384. No 400s !!")
                print('-------------------')
            elif 400 <= id_int <= 399 + int(ProtLocationCount/2):
                print('for count '+str(ProtLocationCount)+'--------current id is'+str(id_int))
                
                A_str = str((id_int-400) * 2)
                if len(A_str)==1:
                    A_str = '00'+A_str
                elif len(A_str) == 2:
                    A_str = '0'+A_str
            
                B_str = str((id_int-400) * 2 +1)
                if len(B_str) == 1:
                    B_str = '00'+B_str
                elif len(B_str) == 2:
                    B_str = '0'+B_str


                if msg['RFC_Loc'+A_str+'_ProtBlockCtr'] == blockCnt:
                    radar_location_tmp_A = Location()
                    radar_location_tmp_B = Location()

                    radar_location_tmp_A.radial_distance = msg['RFC_Loc'+A_str+'_Dr']
                    radar_location_tmp_A.radial_distance_variance = msg['RFC_Loc'+A_str+'_DrVnce']
                    radar_location_tmp_A.radial_distance_spread = 0.0
                    radar_location_tmp_A.radial_velocity = msg['RFC_Loc'+A_str+'_Vr']
                    radar_location_tmp_A.radial_velocity_variance = msg['RFC_Loc'+A_str+'_VrVnce']
                    radar_location_tmp_A.radial_velocity_spread = 0.0
                    radar_location_tmp_A.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_A.radial_distance_velocity_quality = 0.0
                    radar_location_tmp_A.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_A.elevation_angle = math.radians(msg['RFC_Loc'+A_str+'_Phi']) #csa
                    radar_location_tmp_A.elevation_angle_quality = 1.0
                    radar_location_tmp_A.elevation_angle_variance = msg['RFC_Loc'+A_str+'_PhiVnce']
                    radar_location_tmp_A.azimuth_angle = math.radians(msg['RFC_Loc'+A_str+'_Theta']) # csa
                    radar_location_tmp_A.azimuth_angle_quality = 1.0
                    radar_location_tmp_A.azimuth_angle_variance = msg['RFC_Loc'+A_str+'_ThetaVnce']
                    radar_location_tmp_A.rcs = msg['RFC_Loc'+A_str+'_RCS']
                    radar_location_tmp_A.rssi = msg['RFC_Loc'+A_str+'_RSSI']
                    radar_location_tmp_A.measurement_status = 1

                    radar_location_tmp_B.radial_distance = msg['RFC_Loc'+B_str+'_Dr']
                    radar_location_tmp_B.radial_distance_variance = msg['RFC_Loc'+B_str+'_DrVnce']
                    radar_location_tmp_B.radial_distance_spread = 0.0
                    radar_location_tmp_B.radial_velocity = msg['RFC_Loc'+B_str+'_Vr']
                    radar_location_tmp_B.radial_velocity_variance = msg['RFC_Loc'+B_str+'_VrVnce']
                    radar_location_tmp_B.radial_velocity_spread = 0.0
                    radar_location_tmp_B.radial_distance_velocity_covariance = 0.0
                    radar_location_tmp_B.radial_distance_velocity_quality = 0.0
                    radar_location_tmp_B.radial_distance_velocity_spread_orientation = 0.0
                    radar_location_tmp_B.elevation_angle = math.radians(msg['RFC_Loc'+B_str+'_Phi']) #lnl
                    radar_location_tmp_B.elevation_angle_quality = 1.0
                    radar_location_tmp_B.elevation_angle_variance = msg['RFC_Loc'+B_str+'_PhiVnce']
                    radar_location_tmp_B.azimuth_angle = math.radians(msg['RFC_Loc'+B_str+'_Theta']) #lnl
                    radar_location_tmp_B.azimuth_angle_quality = 1.0
                    radar_location_tmp_B.azimuth_angle_variance = msg['RFC_Loc'+B_str+'_ThetaVnce']
                    radar_location_tmp_B.rcs = msg['RFC_Loc'+B_str+'_RCS']
                    radar_location_tmp_B.rssi = msg['RFC_Loc'+B_str+'_RSSI']
                    radar_location_tmp_B.measurement_status = 1

                    loca_interf.raw_locations.append(radar_location_tmp_A)
                    loca_interf.raw_locations.append(radar_location_tmp_B)
                #else:
                    #print("blockcnt doesn't match frame_cnt")
                    #print("blockcnt: {}".format(blockCnt))
                    #print("RFC_Loc_a_protblockctr:{}".format(str(msg['RFC_Loc'+A_str+'_ProtBlockCtr'])))



    def loca_transfer(self, loca_interf):
        point_array = []
        # pub3 = rospy.Publisher("/filtered_output", filtered_location)
        # pub4 = rospy.Publisher("/filtered_LocationInterface",Location)
        # distance = mdf.get('RFC_Loc000_Dr')

        location = loca_interf.raw_locations
        my_point = []
        all_point = []
        header = std_msgs.msg.Header()
        for i in range(len(location)):
            # loca_interf.raw_locations[i].azimuth_angle = math.radians(loca_interf.raw_locations[i].azimuth_angle)
            # loca_interf.raw_locations[i].elevation_angle = math.radians(loca_interf.raw_locations[i].elevation_angle)
            Vel = loca_interf.raw_locations[i].radial_velocity
            distance = loca_interf.raw_locations[i].radial_distance
            # sjy todo math.radians()
            Theta = loca_interf.raw_locations[i].azimuth_angle
            Phi = loca_interf.raw_locations[i].elevation_angle
            x = distance * math.cos(math.radians(Phi)) * math.cos(Theta)
            y = distance * math.cos(math.radians(Phi)) * math.sin(Theta)
            all_point.append([x,y,0.0])
            if Vel > 0.3 or Vel < -0.3:#sjy  filter out objects with low absolute velocity

                # Phi = loca_interf.raw_locations[i].elevation_angle
                # print("Distance:" + str(distance))
                # print("Phi:"+str(Phi))
                # print("Vel:"+str(Vel))
                # Theta = loca_interf.raw_locations[i].azimuth_angle
                point_array.append(distance)
                # sjy add a new output to store the valid filtered output information
                # filtered_output = filtered_location()
                # filtered_output.distance = distance
                # filtered_output.velocity = Vel
                # filtered_output.phi = Phi
                # pub3.publish(filtered_output)
                #self.get_logger().info("distance:%s, vel:%s, phi:%s" % (str(distance),str(Vel),str(Phi)))
                # filtered_loca = Location()    
                # filtered_loca = loca_interf.raw_locations[i]
                # pub4.publish(filtered_loca)
                #x = distance * math.cos(Phi) * math.cos(Theta)
                #y = distance * math.cos(Phi) * math.sin(Theta)
                one_point = [x, y, 0.0]
                my_point.append(one_point)

            # radar_timestamp = time_stamp_radar-3394004
            # test = time.mktime(now.timetuple())
            # radar_time = test + radar_timestamp * 1e-6
            #  t = rospy.Time.from_sec(radar_time)
            # print(radar_time)
            # rospy.loginfo((radar_timestamp - 3394004)/100000)
            # rospy.loginfo(t)

        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        scaled_pcl = pcl12.create_cloud_xyz32(header, my_point)
        #print("pointcloud count: {}, Protlocationcount: {}".format(len(all_point), str(ProtLocationCount)))
        all_pcl = pcl12.create_cloud_xyz32(header,all_point)
        self.pub_2.publish(scaled_pcl)
        self.pub_3.publish(all_pcl)
        print("publish all point")


def main(args=None):

    rclpy.init(args=args)

    readcanNode = readcan_node()

    rclpy.spin(readcanNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    readcanNode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    print("2")
    main()
