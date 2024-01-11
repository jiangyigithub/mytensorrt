import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    urdf = os.path.join(
        get_package_share_directory('radar_object_tracking'),
        'config',
        'szh_vvr_09_12_2020.urdf.xml'
    )

    config = os.path.join(
        get_package_share_directory('radar_object_tracking'),
        'config',
        'test.yaml'
    )

    node_urdf = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_tf_static': True}],
        arguments=[urdf])

    node_tracker = Node(
        package='radar_object_tracking',
        name='radar_tracker',
        node_executable='radar_object_tracking_node',
        output='screen',
        parameters=[config]
        # arguments=['--ros-args','--log-level','DEBUG']
        )

    ld = LaunchDescription()

    ld.add_action(node_urdf)
    ld.add_action(node_tracker)

    return ld
