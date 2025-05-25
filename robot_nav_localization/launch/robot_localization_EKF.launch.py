import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable, NotSubstitution
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_TopicBasedSystem_hardware_interface = LaunchConfiguration('use_TopicBasedSystem_hardware_interface')

    # Specify directory and path to file within package
    robot_localization_pkg_dir = get_package_share_directory('robot_nav_localization')
    laser_odometry_launch_file_subpath = 'launch/laser_odometry.launch.py'
    laser_and_optical_odom_configured_params = 'params/ekf_lidar_and_optical.yaml'


    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'use_TopicBasedSystem_hardware_interface',
            default_value='False',
            description='Use TopicBasedSystem_hardware_interface if true and use GazeboSystem_hardware_interface if false'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_localization_pkg_dir, laser_odometry_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_TopicBasedSystem_hardware_interface': use_TopicBasedSystem_hardware_interface,
                             }.items()
        ),


        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[os.path.join(robot_localization_pkg_dir, laser_and_optical_odom_configured_params)],
                )
            ]
        ),
        

    ])