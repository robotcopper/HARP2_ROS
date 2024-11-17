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
    use_twist_acc_filter = LaunchConfiguration('use_twist_acc_filter')

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_controller_pkg_dir = get_package_share_directory('robot_controller')
    controller_launch_file_subpath = 'launch/controller.launch.py'
    robot_description_launch_file_subpath = 'launch/robot_description.launch.py'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_controller:=', 'True', ' use_TopicBasedSystem_hardware_interface:=', 'True']) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

         DeclareLaunchArgument(
            'use_twist_acc_filter',
            default_value='False',
            description='use_twist_acc_filter'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/pico_mobile_base'
            ],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg_dir, robot_description_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': 'False',
                              'use_robot_state_pub':'True',
                              'use_joint_state_pub': 'True',
                              'use_joint_state_pub_gui': 'False',
                              'robot_description': robot_description_raw
                             }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_controller_pkg_dir, controller_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_TopicBasedSystem_hardware_interface': 'True',
                             }.items()
        ),

        ExecuteProcess(
            condition=IfCondition(use_twist_acc_filter), 
            cmd=[
                'gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '--remap', '/cmd_vel:=/keyboard_cmd_vel'
            ],
            output='screen',
        ),


        ExecuteProcess(
            condition=UnlessCondition(use_twist_acc_filter), 
            cmd=[
                'gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '--remap', '/cmd_vel:=/omnidirectional_controller/cmd_vel_unstamped'
            ],
            output='screen',
        ),

        Node(
            condition=IfCondition(use_twist_acc_filter), 
            package="robot_controller",
            executable="twist_acc_filter",
            output="screen",
            namespace=namespace,
        ),
        

    ])