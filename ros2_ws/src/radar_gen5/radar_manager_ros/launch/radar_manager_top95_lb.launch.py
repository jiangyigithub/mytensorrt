import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config_devices = os.path.join(
        get_package_share_directory('radar_manager_ros'),
        'params',
        'radar_manager_top95_lb_ros2.yaml'
        )

    config_devices_radar_02 = os.path.join(
        get_package_share_directory('radar_manager_ros'),
        'params',
        'radar_manager_top95_lb_ros2_radar02.yaml'
        )

    if os.path.exists(config_devices) == 0:
        print("Error: Config file does not exists. Searched at: " + config_devices)
        sys.exit(-1)
		
    node = Node(
        package = 'radar_manager_ros',
        node_namespace = 'radar_ns',
        node_executable = 'radar_manager_ros',
        parameters = [config_devices],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[
            ('/sensors/radar_gen5/radar_manager_gen5/RadarOutputROB2', '/sensors/radar_01_01/radar_manager_gen5/RadarOutputROB2')]
    )

    node_radar02 = Node(
        package = 'radar_manager_ros',
        node_namespace = 'radar_ns_2',
        node_executable = 'radar_manager_ros',
        parameters = [config_devices_radar_02],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[
        ('/sensors/radar_gen5/radar_manager_gen5/RadarOutputROB2', '/sensors/radar_02_01/radar_manager_gen5/RadarOutputROB2')]
    )

    #ld.add_action(node)
    ld.add_action(node_radar02)
    return ld

