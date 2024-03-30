import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable

import xacro

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_robot_state_pub = 'True'
    use_joint_state_pub = 'False'
    use_joint_state_pub_gui = 'False'
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.04'),}
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    rviz_config_file_subpath = 'rviz/robot_description_rviz.rviz'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    robot_sim_launch_file_subpath = 'launch/robot_sim.launch.py'
    wordl_file_subpath = 'world/Table2024.world'
    
    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control]) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module


    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Launch the user interface window of Gazebo'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='True',
            description='Use ros2_control if true and use Gazebo_control if false'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([robot_sim_pkg_dir, wordl_file_subpath ]),
            description='world file'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='HARP2',
            description='name of the robot'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to start RVIZ'
        ),
            DeclareLaunchArgument(
            'rviz_config_file',
            default_value= PathJoinSubstitution([robot_description_pkg_dir, rviz_config_file_subpath ]),
            description='Full path to the RVIZ config file to use'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_sim_pkg_dir, robot_sim_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'gui': gui,
                              'use_sim_time': use_sim_time,
                              'world': world,
                              'robot_name': robot_name,
                              'robot_description': robot_description_raw,
                              'use_robot_state_pub': use_robot_state_pub,
                              'use_joint_state_pub': use_joint_state_pub,
                              'use_joint_state_pub_gui': use_joint_state_pub_gui,
                              'use_rviz': use_rviz,
                              'rviz_config_file': rviz_config_file,
                              'x_pose': pose['x'],
                              'x_pose': pose['y'],
                              'z_pose': pose['z'],
                             }.items()
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["omnidirectional_controller"],
        ),

    ])