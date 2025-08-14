import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    omo_share = get_package_share_directory('omo_localization')
    params_file = os.path.join(omo_share, 'config', 'ekf_params.yaml')

    ublox_share = get_package_share_directory('ublox_gps')
    ntrip_share = get_package_share_directory('ntrip_client')
    zed_share = get_package_share_directory('zed_wrapper')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ublox_share, 'launch', 'ublox_gps_node-launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ntrip_share, 'launch', 'ntrip_client_launch.py')
            )
        ),
        Node(
            package='ebimu_pkg',
            executable='ebimu_publisher',
            name='ebimu_publisher',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_share, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={'camera_model': 'zed2i'}.items()
        ),
    ])
