import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, NotSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from pathlib import Path

import xacro


def generate_launch_description():
    
	# Create the launch configuration variables    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    pose = {'x': LaunchConfiguration('x_pose'),
            'y': LaunchConfiguration('y_pose'),
            'z': LaunchConfiguration('z_pose'),}
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_joint_state_pub_gui = LaunchConfiguration('use_joint_state_pub_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    robot_description = LaunchConfiguration('robot_description')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_gazebo = LaunchConfiguration('use_gazebo')

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    robot_description_launch_file_subpath = 'launch/robot_description.launch.py'
    rviz_launch_file_subpath = 'launch/rviz.launch.py'
    rviz_config_file_subpath = 'rviz/robot_description_rviz.rviz'
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    wordl_file_subpath = 'world/Table2024.world'
    robot_sim_launch_file_subpath = 'launch/robot_sim.launch.py'

    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = Command(['xacro ', xacro_file, ' use_controller:=', 'False', ' use_TopicBasedSystem_hardware_interface:=', 'False']) # Use of Command to replace robot_description assignment with a call to xacro process rather than Python module

    # Gazebo environment variables setup 
    gazebo_models_world_path = os.path.join(robot_sim_pkg_dir, 'world')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_world_path # Specification of additional model path to use "model://" in world file

    return LaunchDescription([
        
        # Declare launch arguments
	    DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='True',
            description='Launch the user interface window of Gazebo'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
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
            'robot_description',
            default_value=robot_description_raw,
            description='robot_description'
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher'
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub',
            default_value='True',
            description='Whether to start the joint state publisher'
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub_gui',
            default_value='False',
            description='Whether to start the joint state publisher GUI'
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
        DeclareLaunchArgument(
            'x_pose',
            default_value= '0.0',
            description='x_pose'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value= '0.0',
            description='Y_pose'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value= '0.04',
            description='Z_pose'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='True',
            description='Use Gazebo'
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


    ])
