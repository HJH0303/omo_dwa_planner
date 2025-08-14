import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    omo_share = get_package_share_directory('omo_localization')
    params_file = os.path.join(omo_share, 'config', 'ekf_params.yaml')

    ublox_share = get_package_share_directory('ublox_gps')
    ntrip_share = get_package_share_directory('ntrip_client')
    zed_share = get_package_share_directory('zed_wrapper')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')

    # ---- Static TF config (edit here) ----
    parent_frame = 'base_link'
    child_frame  = 'zed_camera_link'
    x, y, z      = '0.0', '0.0', '0.15'
    yaw, pitch, roll = '0.0', '0.0', '0.0'   # radians


    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(ublox_share, 'launch', 'ublox_gps_node-launch.py')
        #     )
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(ntrip_share, 'launch', 'ntrip_client_launch.py')
        #     )
        # ),
        
        Node(
            package='sensor_bringup',
            executable='pcl_voxelization_node',
            name='pcl_voxelization_node',
            output='screen'
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
            )
        ),

        # base_link -> zed_camera_link static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed_tf',
            output='screen',
            arguments=[x, y, z, yaw, pitch, roll, parent_frame, child_frame],
        ),

    ])
