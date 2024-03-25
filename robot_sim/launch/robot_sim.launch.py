import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    
	# Create the launch configuration variables    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    gui_required = LaunchConfiguration('gui_required')
    server_required = LaunchConfiguration('server_required')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.04'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Specify the package directory and path to file within the package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    sdf_file_subpath = 'urdf/robot.sdf'
    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    wordl_file_subpath = 'world/Table2024.world'
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    # Environment variables setup normally optional 
    gazebo_models_world_path = os.path.join(robot_sim_pkg_dir, 'world')
    gazebo_models_robot_description_path = os.path.join(robot_description_pkg_dir, 'urdf')
    gazebo_models_default_path = os.path.join('$HOME', '.gazebo', 'models')
    SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[gazebo_models_world_path, gazebo_models_robot_description_path, gazebo_models_default_path])
    SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH',value=[gazebo_models_world_path, gazebo_models_robot_description_path, gazebo_models_default_path])
    SetEnvironmentVariable(name='GAZEBO_MEDIA_PATH',value=[gazebo_models_world_path, gazebo_models_robot_description_path, gazebo_models_default_path])
    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    SetEnvironmentVariable(name='GAZEBO_MODEL_URI',value=[''])
    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI',value=[''])


    return LaunchDescription([
        
	    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
        ),
        DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Launch the user interface window of Gazebo'
        ),
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
        ),
	    DeclareLaunchArgument(
        'verbose',
        default_value='False',
        description='Run gzserver and gzclient with --verbose, printing errors and warnings to the terminal'
        ),
	    DeclareLaunchArgument(
        'gui_required',
        default_value='True',
        description='Terminate launch script when gzclient (user interface window) exits'
        ),
	    DeclareLaunchArgument(
        'server_required',
        default_value='False',
        description='Terminate launch script when gzserver (Gazebo Server) exits'
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


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world,
                              'use_sim_time': use_sim_time,
                              'gui': gui,
                              'verbose': verbose,
                              'gui_required': gui_required,
                              'server_required': server_required,
                             }.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output={'both': 'log'},
            arguments=[
                '-entity', robot_name,
                #'-topic', 'robot_description',
                '-file', PathJoinSubstitution([robot_description_pkg_dir, sdf_file_subpath]),
                '-robot_namespace', namespace,
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
        )
	    
    ])
