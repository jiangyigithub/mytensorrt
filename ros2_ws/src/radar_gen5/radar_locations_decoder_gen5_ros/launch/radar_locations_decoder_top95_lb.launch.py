import os
import os.path
import sys
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

	# Radar Locations decoder
    config = os.path.join(
        get_package_share_directory('radar_locations_decoder_gen5_ros'),
        'params',
        'radar_locations_decoder_top95_lb_ros2.yaml'
        )

    config2 = os.path.join(
        get_package_share_directory('radar_locations_decoder_gen5_ros'),
        'params',
        'radar_locations_decoder_top95_lb_ros2_radar2.yaml'
        )

    # if os.path.exists(config) == 0:
    #     print("Error: Config file does not exists. Searched at: " + config)
    #     sys.exit(-1)

    node = Node(
        package = 'radar_locations_decoder_gen5_ros',
        node_namespace = 'radar_ns',
        node_executable = 'radar_locations_decoder_gen5_ros',
        parameters = [config],
        output='screen',
        #arguments=['--ros-args', '--log-level', 'DEBUG']
        remappings=[("/sensors/radar_gen5/radar_manager_gen5/RadarOutputROB2","/sensors/radar_01_01/radar_manager_gen5/RadarOutputROB2"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_interface","/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface")]
    )

    node_radar02 = Node(
        package = 'radar_locations_decoder_gen5_ros',
        node_namespace = 'radar_ns_2',
        node_executable = 'radar_locations_decoder_gen5_ros',
        parameters = [config2],
        output='screen',
        #arguments=['--ros-args', '--log-level', 'DEBUG']
        remappings=[("/sensors/radar_gen5/radar_manager_gen5/RadarOutputROB2","/sensors/radar_02_01/radar_manager_gen5/RadarOutputROB2"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_interface","/sensors/radar_02_01/radar_locations_decoder_gen5/location_interface")]
    )

    ld.add_action(node)
    ld.add_action(node_radar02)
    return ld

