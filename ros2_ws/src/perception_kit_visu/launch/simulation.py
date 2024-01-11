from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_kit_visu',
            #remove node_ for foxy and newer
            #node_namespace='turtlesim1',      
            node_executable='perception_kit_visu_object_simulation',
            node_name='object_simulation'
        ),
        Node(
            package='rviz2',
            #remove node_ for foxy and newer
            #node_namespace='turtlesim2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d /home/tobias/workspaces/ros2_ws/src/perception_kit_visu/launch/simulation.rviz']
            #arguments=['-d $(find perception_kit_visu)/launch/simulation.rviz']
        )
    ])