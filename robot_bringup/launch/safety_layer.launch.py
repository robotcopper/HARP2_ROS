import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_bringup')
    yaml_path = os.path.join(pkg_share, 'params', 'collision_monitor.yaml')

    safety_enabled = LaunchConfiguration('safety_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'safety_enabled', default_value='True',
            description=(
                'True: collision_monitor filters /cmd_vel into '
                '/omnidirectional_controller/cmd_vel_safety_unstamped. '
                'False: a topic_tools relay forwards /cmd_vel unchanged (transparent).'
            ),
        ),

        Node(
            condition=IfCondition(safety_enabled),
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[yaml_path],
        ),
        Node(
            condition=IfCondition(safety_enabled),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['collision_monitor'],
            }],
        ),

        Node(
            condition=UnlessCondition(safety_enabled),
            package='topic_tools',
            executable='relay',
            name='cmd_vel_passthrough',
            output='screen',
            arguments=['/cmd_vel', '/omnidirectional_controller/cmd_vel_safety_unstamped'],
        ),
    ])
