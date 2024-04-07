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
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.04'),}
    use_TopicBasedSystem_hardware_interface = LaunchConfiguration('use_TopicBasedSystem_hardware_interface')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = 'True'
    # use_joint_state_pub = 'False'
    use_joint_state_pub_gui = 'False'

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    rviz_config_file_subpath = 'rviz/robot_description_rviz.rviz'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    robot_sim_launch_file_subpath = 'launch/robot_sim.launch.py'
    wordl_file_subpath = 'world/Table2024.world'
    if LaunchConfiguration('use_TopicBasedSystem_hardware_interface'):
        robot_controllers = PathJoinSubstitution([FindPackageShare("robot_controller"),"params","omnidirectional_controller.yaml"])
    else:
        robot_controllers = PathJoinSubstitution([FindPackageShare("robot_controller"),"params","omnidirectional_controller_sim.yaml"])
        
    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_TopicBasedSystem_hardware_interface:=', use_TopicBasedSystem_hardware_interface]) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module

    # reqest contents fo robot_description parameter from robot_state_publisher node
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

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
            'use_TopicBasedSystem_hardware_interface',
            default_value='False',
            description='Use TopicBasedSystem_hardware_interface if true and use GazeboSystem_hardware_interface if false'
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
                              'use_joint_state_pub': use_TopicBasedSystem_hardware_interface,
                              'use_joint_state_pub_gui': use_joint_state_pub_gui,
                              'use_rviz': use_rviz,
                              'rviz_config_file': rviz_config_file,
                              'x_pose': pose['x'],
                              'x_pose': pose['y'],
                              'z_pose': pose['z'],
                              'use_TopicBasedSystem_hardware_interface': use_TopicBasedSystem_hardware_interface,
                              'use_gazebo': NotSubstitution(use_TopicBasedSystem_hardware_interface),
                             }.items()
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
            remappings=[("~/robot_description", "robot_description"),]
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