source /home/icv/Edward/test/sample/install/perception_kit_msgs/share/perception_kit_msgs/local_setup.bash 
source /home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/local_setup.bash 
source /home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/adma_msgs/share/adma_msgs/local_setup.bash 


ros2 bag record /livox/imu_3JEDKBF001T0911 /livox/imu_3WEDJC6001ZE671 /perception/camera/camera_02_02/objects /perception/camera/camera_02_02/objects_transform /perception/camera/camera_02_02/video_data /tf_static /clock /mc_obj_det


ros2 bag record /perception/camera/camera_02_02/objects /perception/camera/camera_02_02/objects_transform /perception/camera/camera_02_02/video_data /tf_static /clock /mc_obj_det


ros2 bag record  /adma_gt_base /AdmaData /livox/lidar_3JEDKBF001T0911 /livox/lidar_3WEDH7600100331 /livox/lidar_3WEDJC6001ZE671 /mc_obj_det /map/groundplane /tf_static 

ros2 bag record  /adma_gt_base /AdmaData /mc_obj_det /map/groundplane /tf_static 

ros2 bag record /livox/lidar_3JEDK4S001F7351 /livox/lidar_3JEDK5A0010P251 /clock /lane_visualization /mc_obj_det /adma_gt_base /mc_obj_track