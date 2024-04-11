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
    if use_TopicBasedSystem_hardware_interface:
        robot_controllers = PathJoinSubstitution([FindPackageShare("robot_controller"),"params","omnidirectional_controller.yaml"])
    else:
        robot_controllers = PathJoinSubstitution([FindPackageShare("robot_controller"),"params","omnidirectional_controller_sim.yaml"])

        
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


        Node(
            package="controller_manager",
            executable="ros2_control_node",
            condition=IfCondition(use_TopicBasedSystem_hardware_interface), # Use ros2_control_node when using TopicBasedSystem because Gazebo plugin not running controller manager for us anymore
            parameters=[{"use_sim_time": False},
                        robot_controllers,
            ],
            output="both",
            namespace=namespace,
            remappings=[("~/robot_description", "robot_description"),] # Use of robot_description topic to get robot description
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            condition=UnlessCondition(use_TopicBasedSystem_hardware_interface), # Use ros2_control_node when using TopicBasedSystem because Gazebo plugin not running controller manager for us anymore
            arguments=["joint_state_broadcaster",
                       "-c", "/controller_manager",
                       "-t", "joint_state_broadcaster/JointStateBroadcaster",
                      ],
            namespace=namespace,
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["omnidirectional_controller",
                       "-c", "/controller_manager",
                       "-t", "omnidirectional_controllers/OmnidirectionalController",
                      ],
            namespace=namespace,
        ),

    ])