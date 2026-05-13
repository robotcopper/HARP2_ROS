import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_params_file = LaunchConfiguration('lidar_params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'params', 'TminiPro.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver. Required for the safety_layer to function. Set to False if /scan is already published by another node (sim, separate bringup).'),
        DeclareLaunchArgument(
            'lidar_params_file', default_value=default_params,
            description='Path to the lidar parameters YAML (robot_bringup/params/TminiPro.yaml or X4.yaml).'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch', 'ydlidar_launch.py')),
            launch_arguments={'params_file': lidar_params_file}.items(),
            condition=IfCondition(use_lidar),
        ),
    ])
