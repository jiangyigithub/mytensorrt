from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    dbc_path_radar3 = os.path.join(
        get_package_share_directory('readcan'),
        'dbc',
        'FR5CP_Radar_LGU.dbc')
    dbc_path_radar2 = os.path.join(
        get_package_share_directory('readcan'),
        'dbc',
        'FR5CP_LGU_v3.0_1xx.dbc')

    dbc_path_radar1 = os.path.join(
        get_package_share_directory('readcan'),
        'dbc',
        'FR5CP_Radar_LGU.dbc')

    readcan_node_radar3 = Node(
        package="readcan",
        node_executable="readcan",
        name="radar_02_02",
        output="screen",
        parameters=[
            {"dbc_path": dbc_path_radar3},  # LGU_dbc.dbc
            {"frame_id": "radar_02_02"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_02_02/radar_locations_decoder_gen5/location_interface"),
                    ("tcp/canfd", "/tcp/radar_02_02/canfd"),
                    ("/all_cloud", "/radar_02_02/all_cloud"),
                    ("/filtered_clound", "/radar_02_02/filtered_clound")
                    ]
    )

    readcan_node_radar2 = Node(
        package="readcan",
        node_executable="readcan",
        name="radar_02_01",
        output="screen",
        parameters=[
            # LGU_dbc.dbc
            {"dbc_path": dbc_path_radar2},
            {"frame_id": "radar_02_01"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_02_01/radar_locations_decoder_gen5/location_interface"),  # /LocationInterface","/sensors/radar_02_01_canfd/radar_locations_decoder_gen5/location_interface
                    ("tcp/canfd", "/tcp/radar_02_01/canfd"),
                    ("/all_cloud", "/radar_02_01/all_cloud"),
                    ("/filtered_clound", "/radar_02_01/filtered_clound")
                    ]
    )
    readcan_node_radar1 = Node(
        package="readcan",
        node_executable="readcan",
        name="radar_01_01",
        output="screen",
        parameters=[
            {"dbc_path": dbc_path_radar1},
            {"frame_id": "radar_01_01"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface"),
                    ("tcp/canfd", "/tcp/radar_01_01/canfd"),
                    ("/all_cloud", "/radar_01_01/all_cloud"),
                    ("/filtered_clound", "/radar_01_01/filtered_clound")
                    ]
    )

    # cam_tf2_ros_cmd = Node(
    #     package = "tf2_ros",
    #     node_executable = "static_transform_publisher",
    #     name = "camera_tf",
    #     arguments = ['0', '0', '0', '0', '0', '0', 'map', 'usb_cam']
    # )

    # radar_tf2_ros_cmd = Node(
    #     package = "tf2_ros",
    #     node_executable =  "static_transform_publisher",
    #     name = "radar_tf",
    #     arguments = ['0', '0', '0', '0', '0', '0', 'map', 'radar_link']
    # )

    # rviz2_cmd = Node(
    #     package = "rviz2",
    #     node_executable = "rviz2",
    #     name = "rviz2",
    #     arguments = ['-d /home/icv2/ros2_ws/src/ethernetcan-python/can_read.rviz']
    # )

    ld.add_action(readcan_node_radar1)
    ld.add_action(readcan_node_radar2)
    ld.add_action(readcan_node_radar3)
    # ld.add_action(cam_tf2_ros_cmd)
    # ld.add_action(radar_tf2_ros_cmd)
    # ld.add_action(rviz2_cmd)
    return ld
