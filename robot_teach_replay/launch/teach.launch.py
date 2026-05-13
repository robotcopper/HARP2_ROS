import os
import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_bringup_pkg = get_package_share_directory('robot_bringup')

    bag_root = os.path.expanduser('~/ros2_ws/recorded_trajectories')
    os.makedirs(bag_root, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    default_bag_path = os.path.join(bag_root, f'teach_{timestamp}')

    namespace = LaunchConfiguration('namespace')
    launch_on_robot = LaunchConfiguration('launch_on_robot')
    safety_enabled = LaunchConfiguration('safety_enabled')
    use_lidar = LaunchConfiguration('use_lidar')
    bag_path = LaunchConfiguration('bag_path')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'launch_on_robot', default_value='False',
            description='Launch micro_ros_agent on /dev/pico_mobile_base'),
        DeclareLaunchArgument(
            'safety_enabled', default_value='True',
            description='Enable collision_monitor safety filter on /cmd_vel'),
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver (required for safety filter)'),
        DeclareLaunchArgument(
            'bag_path', default_value=default_bag_path,
            description='Output directory for the recorded bag'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_pkg, 'launch', 'gamepad_control.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'launch_on_robot': launch_on_robot,
                'safety_enabled': safety_enabled,
                'use_lidar': use_lidar,
            }.items(),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path, '/cmd_vel'],
            output='screen',
        ),
    ])
