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

    # Specify directory and path to file within package
    robot_bringup_pkg_dir = get_package_share_directory('robot_bringup')
    nav2_bringup_launch_file_subpath = 'launch/nav2_bringup.launch.py'



    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'
            ],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_pkg_dir, nav2_bringup_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_TopicBasedSystem_hardware_interface': 'True',
                              'use_gazebo_gui': 'True',
                              'use_rviz': 'False',
                             }.items()
        ),

        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '--remap', '/cmd_vel:=/omnidirectional_controller/cmd_vel_unstamped'
            ],
            output='screen'
        ),

        # ExecuteProcess(
        #     cmd=[
        #         'terminator', '-e', 
        #         'bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/omnidirectional_controller/cmd_vel_unstamped; exec bash"'
        #     ],
        #     output='screen'
        # ),

    ])