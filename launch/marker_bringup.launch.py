import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sensor_bringup',
            executable='segformer_node.py',
            name='segformer_node',
            output='screen'
        ),
        Node(
            package='sensor_bringup',
            executable='visual_marking_global_prj.py',
            name='visual_marker',
            output='screen'
        ),
        Node(
            package='sensor_bringup',
            executable='visual_marking.py',
            name='visual_marker',
            output='screen'
        ),
        Node(
            package='sensor_bringup',
            executable='mask_ransac_pcl_node',
            name='mask_ransac_pcl_node',
            output='screen'
        ),
        Node(
            package='sensor_bringup',
            executable='pcl_voxelization_node',
            name='pcl_voxelization_node',
            output='screen'
        ),
    ])
