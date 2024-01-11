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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    # ================ Argument ================ #
    Argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True')

# -----------------------------------------------------------------------------
#   Config
# -----------------------------------------------------------------------------
    # ================ Config: radar readcan ================ #
    config_dbc_path_radar4 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'readcan',
        'dbc',
        'FR5CP_LGU_v3.0_1xx.dbc')
    config_dbc_path_radar3 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'readcan',
        'dbc',
        'FR5CP_LGU_v3.0_1xx.dbc')
    config_dbc_path_radar2 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'readcan',
        'dbc',
        'FR5CP_LGU_v3.0_1xx.dbc')

    config_dbc_path_radar1 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'readcan',
        'dbc',
        'FR5CP_LGU_v3.0_1xx.dbc')

    # ================ Config: radar tracker ================ #
    config_radar_01_01 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_gen5',
        'radar_object_tracker.yaml')
    config_radar_02_01 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_gen5',
        'radar_object_tracker_2.yaml')
    config_radar_03_01 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_gen5',
        'radar_object_tracker_3.yaml')
    config_radar_04_01 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_gen5',
        'radar_object_tracker_4.yaml') 

    # ================ Config: urdf & transform ================ #
    urdf_file_name = 'wuxi_houshan.urdf.xml'
    # urdf_file_name = 'cr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('top95_general'),
        'urdf',
        urdf_file_name)

    # ================ Config: t2t ================ #
    config_t2t_weights = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        't2t',
        'weights.yaml')
    config_t2t_input = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        't2t',
        'input_offline.yaml')


	# ================ Config: radar pointcloud ================ #
    config_radar_pointcloud_radar1 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_pointcloud',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2.yaml'
        )
    config_radar_pointcloud_radar2 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_pointcloud',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_2.yaml'
        )
    config_radar_pointcloud_radar3 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_pointcloud',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_3.yaml'
        )
    config_radar_pointcloud_radar4 = os.path.join(
        get_package_share_directory('top95_general'),
        'config',
        'radar_pointcloud',
        'radar_locations_to_pointcloud_converter_top95_lb_ros2_4.yaml'
        )
# -----------------------------------------------------------------------------
#   Node
# -----------------------------------------------------------------------------
    # ================ Node: readcan ================ #
    readcan_node_radar4 = Node(
        package="readcan",
        executable="readcan",
        name="readcan_radar_04_01",
        output="log",
        parameters=[
            # LGU_dbc.dbc
            {"dbc_path": config_dbc_path_radar4},
            {"frame_id": "radar_04_01"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_04_01/radar_locations_decoder_gen5/location_interface"),
                    ("tcp/canfd", "/tcp/radar_04_01/canfd"),
                    ("/all_cloud", "/radar_04_01/all_cloud"),
                    ("/filtered_clound", "/radar_04_01/filtered_clound")
                    ]
    )

    readcan_node_radar3 = Node(
        package="readcan",
        executable="readcan",
        name="readcan_radar_03_01",
        output="log",
        parameters=[
            # LGU_dbc.dbc
            {"dbc_path": config_dbc_path_radar3},
            {"frame_id": "radar_03_01"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_03_01/radar_locations_decoder_gen5/location_interface"),
                    ("tcp/canfd", "/tcp/radar_03_01/canfd"),
                    ("/all_cloud", "/radar_03_01/all_cloud"),
                    ("/filtered_clound", "/radar_03_01/filtered_clound")
                    ]
    )

    readcan_node_radar2 = Node(
        package="readcan",
        executable="readcan",
        name="readcan_radar_02_01",
        output="log",
        parameters=[
            # LGU_dbc.dbc
            {"dbc_path": config_dbc_path_radar2},
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
        executable="readcan",
        name="readcan_radar_01_01",
        output="log",
        parameters=[
            {"dbc_path": config_dbc_path_radar1},
            {"frame_id": "radar_01_01"}
        ],
        remappings=[("/LocationInterface", "/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface"),
                    ("tcp/canfd", "/tcp/radar_01_01/canfd"),
                    ("/all_cloud", "/radar_01_01/all_cloud"),
                    ("/filtered_clound", "/radar_01_01/filtered_clound")
                    ]
    )

    # ================ Node: radar tracker ================ #
    tracker_node_radar1 = Node(
        package='radar_object_tracking',
        # node_namespace='radar_ns',
        name="Rtracker_radar_01_01",
        executable='radar_object_tracking_node',
        parameters=[config_radar_01_01],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface", "/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface"),
                    ("/tracks_ros2_test", "/radar_01_01/tracks_ros2")])
    tracker_node_radar2 = Node(
        package='radar_object_tracking',
        # node_namespace='radar_ns',
        name="Rtracker_radar_02_01",
        executable='radar_object_tracking_node',
        parameters=[config_radar_02_01],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface", "/sensors/radar_02_01/radar_locations_decoder_gen5/location_interface"),
                    ("/tracks_ros2_test", "radar_02_01/tracks_ros2")])
    tracker_node_radar3 = Node(
        package='radar_object_tracking',
        # node_namespace='radar_ns',
        name="Rtracker_radar_03_01",
        executable='radar_object_tracking_node',
        parameters=[config_radar_03_01],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface", "/sensors/radar_03_01/radar_locations_decoder_gen5/location_interface"),
                    ("/tracks_ros2_test", "radar_03_01/tracks_ros2")])
    tracker_node_radar4 = Node(
        package='radar_object_tracking',
        # node_namespace='radar_ns',
        name="Rtracker_radar_04_01",
        executable='radar_object_tracking_node',
        parameters=[config_radar_04_01],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface", "/sensors/radar_04_01/radar_locations_decoder_gen5/location_interface"),
                    ("/tracks_ros2_test", "radar_04_01/tracks_ros2")])

    # ================ Node: urdf&transform ================ #

    urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='urdf',
        output='log',
        parameters=[{'use_tf_static': True}],
        arguments=[urdf])

    transform_node = ComposableNodeContainer(
        node_name='transform',
        node_namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_01_01',
                parameters=[
                    {'input/frame': ''},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/mc_obj_det'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_01_01/objects_transform'}
                ]),
            # ComposableNode(
            #     package='perception_kit_object_transform',
            #     node_plugin='perception_kit::object_transform::ObjectTransform',
            #     node_name='tr_camera_01_01',
            #     parameters=[
            #         {'input/frame': 'camera_01_01'},
            #         {'output/frame': 'layered_map_enu'},
            #         {'output/target_frame_needs_motion': False},
            #         {'input/topic': '/perception/camera/camera_01_01/objects'},
            #         {'output/topic_needs_common': False},
            #         {'output/topic_common': '/perception/camera/objects_transform'},
            #         {'output/topic': '/perception/camera/camera_01_01/objects_transform'}
            #     ]),
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_02_01',
                parameters=[
                    {'input/frame': 'camera_02_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_02_01/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_02_01/objects_transform'}
                ]),

            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_03_01',
                parameters=[
                    {'input/frame': 'camera_03_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_03_01/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_03_01/objects_transform'}
                ]),

            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_04_01',
                parameters=[
                    {'input/frame': 'camera_04_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_04_01/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_04_01/objects_transform'}
                ]),

            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_01_02',
                parameters=[
                    {'input/frame': 'camera_01_02'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_01_02/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_01_02/objects_transform'}
                ]),
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_02_02',
                parameters=[
                    {'input/frame': 'camera_02_02'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_02_02/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_02_02/objects_transform'}
                ]),

            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_03_02',
                parameters=[
                    {'input/frame': 'camera_03_02'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_03_02/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_03_02/objects_transform'}
                ]),

            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_camera_04_02',
                parameters=[
                    {'input/frame': 'camera_04_02'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': '/perception/camera/camera_04_02/objects'},
                    {'output/topic_needs_common': False},
                    {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/camera/camera_04_02/objects_transform'}
                ]), 

                
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_radar_01_01',
                parameters=[
                    {'input/frame': 'radar_01_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': 'radar_01_01/tracks_ros2'},
                    {'output/topic_needs_common': False},
                    # {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/radar_01_01/tracks_transform_ros2'}
                ]),
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_radar_02_01',
                parameters=[
                    {'input/frame': 'radar_02_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': 'radar_02_01/tracks_ros2'},
                    {'output/topic_needs_common': False},
                    # {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/radar_02_01/tracks_transform_ros2'}
                ]),
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_radar_03_01',
                parameters=[
                    {'input/frame': 'radar_03_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': 'radar_03_01/tracks_ros2'},
                    {'output/topic_needs_common': False},
                    # {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/radar_03_01/tracks_transform_ros2'}
                ]),
            ComposableNode(
                package='perception_kit_object_transform',
                node_plugin='perception_kit::object_transform::ObjectTransform',
                node_name='tr_radar_04_01',
                parameters=[
                    {'input/frame': 'radar_04_01'},
                    {'output/frame': 'layered_map_enu'},
                    {'output/target_frame_needs_motion': False},
                    {'input/topic': 'radar_04_01/tracks_ros2'},
                    {'output/topic_needs_common': False},
                    # {'output/topic_common': '/perception/camera/objects_transform'},
                    {'output/topic': '/perception/radar_04_01/tracks_transform_ros2'}
                ])
            ########Radar test ######
#            ComposableNode(
#                package='perception_kit_object_transform',
#                node_plugin='perception_kit::object_transform::ObjectTransform',
#                node_name='transform_radar_ros2_test',
#                parameters=[
#                    {'input/frame': 'radar_02_01'},  # camera_01_01
#                    {'output/frame': 'layered_map_enu'},
#                    {'output/target_frame_needs_motion': False},
#                    {'input/topic': '/tracks_ros2_test'},
#                    {'output/topic_needs_common': False},
#                    # {'output/topic_common': '/perception/camera/objects_transform'},
#                    {'output/topic': '/perception/radar/tracks_transform_ros2_test'}
#                ])
        ],
        output='log',
    )

    # ================ Node: T2T ================ #
    t2t_node = Node(
        package='track_to_track_fusion',
        name="T2T",
        executable='track_to_track_fusion_node',
        parameters=[
            config_t2t_input,
            config_t2t_weights],
        output='log')

    # ================ Node: radar tcp2ros ================ #
    tcp2ros_node_radar4 = Node(
        package="tcp2ros",
        name="tcp2ros_radar_04_01",
        executable="tcp2ros",
        output="log",
        arguments=["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            # 192.168.0.168  pole3->1?,  192.168.0.178 pole1->old radar 192.168.0.158 on creation
            {"ip": "192.168.5.70"}
        ],
        remappings=[("tcp/canfd", "/tcp/radar_04_01/canfd")],
        on_exit=actions.Shutdown()
    )

    tcp2ros_node_radar3 = Node(
        package="tcp2ros",
        name="tcp2ros_radar_03_01",
        executable="tcp2ros",
        output="log",
        arguments=["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            # 192.168.0.168  pole3->1?,  192.168.0.178 pole1->old radar 192.168.0.158 on creation
            {"ip": "192.168.5.69"}
        ],
        remappings=[("tcp/canfd", "/tcp/radar_03_01/canfd")],
        on_exit=actions.Shutdown()
    )
    tcp2ros_node_radar2 = Node(
        package="tcp2ros",
        name="tcp2ros_radar_02_01",
        executable="tcp2ros",
        output="log",
        arguments=["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            # 192.168.0.168  pole3->1?,  192.168.0.178 pole1->old radar 192.168.0.158 on creation
            {"ip": "192.168.5.68"}
        ],
        remappings=[("tcp/canfd", "/tcp/radar_02_01/canfd")],
        on_exit=actions.Shutdown()
    )
    tcp2ros_node_radar1 = Node(
        package="tcp2ros",
        name="tcp2ros_radar_01_01",
        executable="tcp2ros",
        output="log",
        arguments=["--number_of_cycles", "1"],
        parameters=[
            {"can_ethernet_model_name": "ZLG"},
            {"ip": "192.168.5.67"}  # 192.168.0.168  pole3,  192.168.0.178 pole1
        ],
        remappings=[("tcp/canfd", "/tcp/radar_01_01/canfd")],
        on_exit=actions.Shutdown()
    )
    # ================ Node: clock ================ #
    clock_node = Node(
        package="clock_publisher",
        name="clock_publisher",
        executable="clock_publisher_node",
        parameters=[
            {"rate": 200}
        ],
        output='log')

    # ================ Node: radar point cloud ================ #
    cloud_node_radar1 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        name = 'pcl_radar_01_01',
        executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_pointcloud_radar1],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_01_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_01_01/radar_locations_decoder_gen5/location_pointcloud")]
 
    )
    cloud_node_radar2 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        name = 'pcl_radar_02_01',
        executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_pointcloud_radar2],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_02_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_02_01/radar_locations_decoder_gen5/location_pointcloud")]
    )
    cloud_node_radar3 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        name = 'pcl_radar_03_01',
        executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_pointcloud_radar3],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_03_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_03_01/radar_locations_decoder_gen5/location_pointcloud")]
    )
    cloud_node_radar4 = Node(
        package = 'radar_locations_to_pointcloud_converter_ros',
        name = 'pcl_radar_04_01',
        executable = 'radar_locations_to_pointcloud_converter_ros',
        parameters = [config_radar_pointcloud_radar4],
        output='log',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        remappings=[("/radar_decoder_gen5/location_interface","/sensors/radar_04_01/radar_locations_decoder_gen5/location_interface"),
        ("/sensors/radar_gen5/radar_locations_decoder_gen5/location_pointcloud","/sensors/radar_04_01/radar_locations_decoder_gen5/location_pointcloud")]
    )
    # ================ Node: ffmpeg ================ #
    ffmpeg_node_cam12 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_01_02",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_01_02/video_data"),
        ("/output/video", "/perception/camera/camera_01_02/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam22 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_02_02",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_02_02/video_data"),
        ("/output/video", "/perception/camera/camera_02_02/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam32 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_03_02",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_03_02/video_data"),
        ("/output/video", "/perception/camera/camera_03_02/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam42 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_04_02",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_04_02/video_data"),
        ("/output/video", "/perception/camera/camera_04_02/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    
    ffmpeg_node_cam11 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_01_01",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_01_01/video_data"),
        ("/output/video", "/perception/camera/camera_01_01/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam21 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_02_01",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_02_01/video_data"),
        ("/output/video", "/perception/camera/camera_02_01/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam31 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_03_01",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_03_01/video_data"),
        ("/output/video", "/perception/camera/camera_03_01/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    ffmpeg_node_cam41 = Node(
        package="ffmpeg_decoder_bridge",
        name="ffmpeg_cam_04_01",
        executable="ffmpeg_decoder_bridge_node",
        output="log",
        remappings=[
        ("/input/video", "/perception/camera/camera_04_01/video_data"),
        ("/output/video", "/perception/camera/camera_04_01/video_decoded")
        ],
        on_exit=actions.Shutdown()
    )
    
# -----------------------------------------------------------------------------
#   Action
# -----------------------------------------------------------------------------

    # ================ Action ================ #
    ld.add_action(Argument)
    ld.add_action(urdf_node)
    # readcan
    #ld.add_action(readcan_node_radar1)
    #ld.add_action(readcan_node_radar2)
    #ld.add_action(readcan_node_radar3)
    #ld.add_action(readcan_node_radar4)

    # tracker
    # ld.add_action(tracker_node_radar1)
    # ld.add_action(tracker_node_radar2)
    # ld.add_action(tracker_node_radar3)
    # ld.add_action(tracker_node_radar4)

    # urdf & transform
    
    ld.add_action(transform_node)

    # t2t
    #ld.add_action(t2t_node)

    # tcp2ros
    #ld.add_action(tcp2ros_node_radar1)
    #ld.add_action(tcp2ros_node_radar2)
    #ld.add_action(tcp2ros_node_radar3)
    #ld.add_action(tcp2ros_node_radar4)
    
    # clock
    #ld.add_action(clock_node)

    # radar cloud
    # ld.add_action(cloud_node_radar1)
    # ld.add_action(cloud_node_radar2)
    # ld.add_action(cloud_node_radar3)
    # ld.add_action(cloud_node_radar4)
    
    #ffmpeg
    #ld.add_action(ffmpeg_node_cam11)
    #ld.add_action(ffmpeg_node_cam21)
    #ld.add_action(ffmpeg_node_cam31)
    #ld.add_action(ffmpeg_node_cam41)
    #ld.add_action(ffmpeg_node_cam12)
    # ld.add_action(ffmpeg_node_cam22)
    #ld.add_action(ffmpeg_node_cam32)
    #ld.add_action(ffmpeg_node_cam42)     

    return ld
