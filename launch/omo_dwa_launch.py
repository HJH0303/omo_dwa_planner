import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='omo_dwa_planner',
            executable='dwa_node',
            name='dwa_node',
            output='screen'
        ),
        Node(
            package='omo_dwa_planner',
            executable='carrot_node',
            name='carrot_node',
            output='screen'
        )
    ])
