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
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.04'),}
    use_TopicBasedSystem_hardware_interface = LaunchConfiguration('use_TopicBasedSystem_hardware_interface')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_description = LaunchConfiguration('robot_description')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    map = LaunchConfiguration('map')
    use_robot_state_pub = 'True'
    use_joint_state_pub = LaunchConfiguration('use_TopicBasedSystem_hardware_interface')
    use_joint_state_pub_gui = 'False'
    use_sim_time = NotSubstitution(use_TopicBasedSystem_hardware_interface)
    use_gazebo = NotSubstitution(use_TopicBasedSystem_hardware_interface)

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    robot_controller_pkg_dir = get_package_share_directory('robot_controller')
    robot_bringup_pkg_dir = get_package_share_directory('robot_bringup')
    robot_localization_pkg_dir = get_package_share_directory('robot_localization')
    robot_slam_pkg_dir = get_package_share_directory('robot_slam')
    rviz_config_file_subpath = 'rviz/amcl.rviz'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    robot_sim_launch_file_subpath = 'launch/robot_sim.launch.py'
    controller_launch_file_subpath = 'launch/controller.launch.py'
    robot_description_launch_file_subpath = 'launch/robot_description.launch.py'
    rviz_launch_file_subpath = 'launch/rviz.launch.py'
    laser_odometry_launch_file_subpath = 'launch/laser_odometry.launch.py'
    amcl_launch_file_subpath = 'launch/amcl.launch.py'
    wordl_file_subpath = 'world/Table2024.world'
    amcl_params_file_subpath = 'params/amcl.yaml'
    map_file_subpath = 'maps/Table2024.yaml'

    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_controller:=', 'True', ' use_TopicBasedSystem_hardware_interface:=', use_TopicBasedSystem_hardware_interface]) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module


    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
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
            default_value= PathJoinSubstitution([robot_localization_pkg_dir, rviz_config_file_subpath ]),
            description='Full path to the RVIZ config file to use'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_raw,
            description='robot_description'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='True',
            description='Use Gazebo'
        ),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=PathJoinSubstitution([robot_localization_pkg_dir, amcl_params_file_subpath]),
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([robot_slam_pkg_dir,map_file_subpath]),
            description='map'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg_dir, robot_description_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'use_robot_state_pub': use_robot_state_pub,
                              'use_joint_state_pub': use_joint_state_pub,
                              'use_joint_state_pub_gui': use_joint_state_pub_gui,
                              'robot_description': robot_description
                             }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg_dir, rviz_launch_file_subpath)
            ),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': namespace,
                            'rviz_config': rviz_config_file,
                            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_sim_pkg_dir, robot_sim_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_gazebo_gui': use_gazebo_gui,
                              'use_sim_time': use_sim_time,
                              'world': world,
                              'robot_name': robot_name,
                              'x_pose': pose['x'],
                              'x_pose': pose['y'],
                              'z_pose': pose['z'],
                              'use_gazebo': use_gazebo,
                             }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_controller_pkg_dir, controller_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_TopicBasedSystem_hardware_interface': use_TopicBasedSystem_hardware_interface,
                             }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_pkg_dir, laser_odometry_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_TopicBasedSystem_hardware_interface': use_TopicBasedSystem_hardware_interface,
                             }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_localization_pkg_dir, amcl_launch_file_subpath)
            ),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': amcl_params_file,
                              'map': map,
                             }.items()
        ),

    ])