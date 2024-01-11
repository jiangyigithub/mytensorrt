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
 

from launch import LaunchDescription
from launch_ros.actions import Node
from launch import actions

def generate_launch_description():
    ld = LaunchDescription()
    tcp2ros_node_radar3 = Node(
        package="tcp2ros",
        name="radar_02_02",
        node_executable="tcp2ros",
        output = "screen",
        arguments = ["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            {"ip": "192.168.0.178"}  #192.168.0.168  pole3->1?,  192.168.0.178 pole1->old radar 192.168.0.158 on creation
        ],
        remappings=[("tcp/canfd","/tcp/radar_02_02/canfd")],
        on_exit = actions.Shutdown()
    )
    tcp2ros_node_radar2 = Node(
        package="tcp2ros",
        name="radar_02_01",
        node_executable="tcp2ros",
        output = "screen",
        arguments = ["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            {"ip": "192.168.0.158"}  #192.168.0.168  pole3->1?,  192.168.0.178 pole1->old radar 192.168.0.158 on creation
        ],
        remappings=[("tcp/canfd","/tcp/radar_02_01/canfd")],
        on_exit = actions.Shutdown()
    )
    tcp2ros_node_radar1 = Node(
        package="tcp2ros",
        name="radar_01_01",
        node_executable="tcp2ros",
        output = "screen",
        arguments = ["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            {"ip": "192.168.0.168"}  #192.168.0.168  pole3,  192.168.0.178 pole1
        ],
        remappings=[("tcp/canfd","/tcp/radar_01_01/canfd")],
        on_exit = actions.Shutdown()
    )

    ld.add_action(tcp2ros_node_radar1)
    #ld.add_action(tcp2ros_node_radar2)
    ld.add_action(tcp2ros_node_radar3)
    return ld
