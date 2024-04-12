import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

	# Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    freq = LaunchConfiguration('freq')

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='scan_topic'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom_rf2o',
            description='odom_topic'
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_footprint',
            description='base_frame_id'
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='odom_frame_id'
        ),
        DeclareLaunchArgument(
            'freq',
            default_value='20.0',
            description='freq'
        ),


        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            namespace=namespace,
            parameters=[{
                        'laser_scan_topic' : scan_topic,
                        'odom_topic' : odom_topic,
                        'publish_tf' : True,
                        'base_frame_id' : base_frame_id,
                        'odom_frame_id' : odom_frame_id,
                        'init_pose_from_topic' : '',
                        'freq' : freq,
                       }],
        ),

    ])
