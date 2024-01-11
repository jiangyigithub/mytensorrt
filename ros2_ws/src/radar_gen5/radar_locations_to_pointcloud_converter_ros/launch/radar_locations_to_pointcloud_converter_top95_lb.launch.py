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
	
	# Pointcloud converter
    config = os.path.join(
        get_package_share_directory('radar_locations_to_pointcloud_converter_ros'),
        'params',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2.yaml'
        )
    config_radar_02 = os.path.join(
        get_package_share_directory('radar_locations_to_pointcloud_converter_ros'),
        'params',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_2.yaml'
        )
    config_radar_03 = os.path.join(
        get_package_share_directory('radar_locations_to_pointcloud_converter_ros'),
        'params',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_3.yaml'
        )

    configcanfd = os.path.join(
        get_package_share_directory('radar_locations_to_pointcloud_converter_ros'),
        'params',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2canfd.yaml'
        )
    config_radar_02canfd = os.path.join(
        get_package_share_directory('radar_locations_to_pointcloud_converter_ros'),
        'params',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_2canfd.yaml'
        )

    if os.path.exists(config) == 0:
        print("Error: Config file does not exists. Searched at: " + config)
        sys.exit(-1)

    node = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        node_namespace = 'radar_ns',
        # node_name = "aa",
        node_executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_01_01/radar_locations_decoder_gen5/location_pointcloud")]
 
    )
    node_radar02 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        node_namespace = 'radar_ns_2',
        # node_name = "bb",
        node_executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_02],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_02_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_02_01/radar_locations_decoder_gen5/location_pointcloud")]
    )
    node_radar03 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        node_namespace = 'radar_ns_3',
        # node_name = "bb",
        node_executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_03],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_02_02/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_02_02/radar_locations_decoder_gen5/location_pointcloud")]
    )

    node_can = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        node_namespace = 'radar_nscanfd',
        # node_name = "cc",
        node_executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [configcanfd],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_01_01_canfd/radar_locations_decoder_gen5/location_interface")]#,
       # ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_01_01_canfd/radar_locations_decoder_gen5/location_pointcloud")]
 
    )
    node_radar02_can = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        node_namespace = 'radar_ns_2canfd',
        # node_name = "dd",
        node_executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_02canfd],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_02_01_canfd/radar_locations_decoder_gen5/location_interface")]#,
        #("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_02_01_canfd/radar_locations_decoder_gen5/location_pointcloud")]
    )
    # node_can = Node(
    #     package = 'radar_locations_to_pointcloud_converter_ros',
    #     node_namespace = 'radar_nscanfd',
    #     # node_name = "cc",
    #     node_executable = 'radar_locations_to_pointcloud_converter_ros',
    #     parameters = [configcanfd],
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'DEBUG'],
    #     remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_01_01_canfd/radar_locations_decoder_gen5/location_interface"),
    #     ("/sensors/radar_01_01/radar_locations_decoder_gen5/location_pointcloud/radar_01_01/location_cloud","/ababa")]
 
    # )
    # node_radar02_can = Node(
    #     package = 'radar_locations_to_pointcloud_converter_ros',
    #     node_namespace = 'radar_ns_2canfd',
    #     # node_name = "dd",
    #     node_executable = 'radar_locations_to_pointcloud_converter_ros',
    #     parameters = [config_radar_02canfd],
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', 'DEBUG'],
    #     remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_02_01_canfd/radar_locations_decoder_gen5/location_interface"),
    #     ("/sensors/radar_02_01/radar_locations_decoder_gen5/location_pointcloud/radar_02_01/location_cloud","/010101")]
    # )

    ld.add_action(node)
    ld.add_action(node_radar02)
    ld.add_action(node_can)
    ld.add_action(node_radar02_can)
    ld.add_action(node_radar03)
    return ld