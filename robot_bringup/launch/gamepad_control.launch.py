import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, OpaqueFunction
import xacro
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable, NotSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    launch_on_robot = LaunchConfiguration('launch_on_robot')

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_controller_pkg_dir = get_package_share_directory('robot_controller')
    controller_launch_file_subpath = 'launch/controller.launch.py'
    robot_description_launch_file_subpath = 'launch/robot_description.launch.py'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    gamepad_config = PathJoinSubstitution([FindPackageShare("robot_bringup"),"params","teleop_holonomic_config.yaml"])

    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_controller:=', 'True', ' use_TopicBasedSystem_hardware_interface:=', 'True']) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module

    # Define a function to log a colored message
    def log_message(context):
        # Display the message in green with ANSI escape codes
        print(
            "\033[1;95m" +  # Bold violet
            "\nDon't forget to run:\n" +
            "\033[3m" +  # Italic
            "    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/pico_mobile_base" +
            "\033[0m" +  # Reset italics
            "\033[1;95m" +  # Return to bold violet
            "\non robot side" +
            "\033[0m"  # Reset all styles
        )

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),

        DeclareLaunchArgument(
            'launch_on_robot',
            default_value='False',
            description='launch_on_robot'
        ),

        ExecuteProcess(
            condition=IfCondition(launch_on_robot), 
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

        Node(
            package="joy",
            executable="joy_node",
            output="screen",
            namespace=namespace,
            remappings=[("/joy", "/gamepad_joy"),]
        ),
            
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            parameters=[gamepad_config,
            ],
            output="screen",
            namespace=namespace,
            remappings=[("/cmd_vel", "/omnidirectional_controller/cmd_vel_unstamped"),("/joy", "/gamepad_joy"),]
        ),
        
        # Log the message if launch_on_robot is false
        TimerAction(
            period=2.0,  # Wait 2 seconds after the last node starts
            actions=[
                OpaqueFunction(function=log_message, condition=UnlessCondition(launch_on_robot))
            ]
        ),

    ])