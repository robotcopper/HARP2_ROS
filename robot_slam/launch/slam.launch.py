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
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
	# Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_localization = LaunchConfiguration('use_localization')

    # Specify directory and path to file within package
    slam_params_localization = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), "params", "mapper_params_localization.yaml"]
    )
    slam_params_mapping = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), "params", "mapper_params_online_async.yaml"]
    )

    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_localization',
            default_value='False',
            description='Whether to use localization mode with slam_toolbox'
        ),

        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("slam_toolbox"), 'launch', 'online_async_launch.py')
            ),
            condition=UnlessCondition(use_localization),  
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_mapping,
            }.items()
        ),

        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("slam_toolbox"), 'launch', 'localization_launch.py')
            ),
            condition=IfCondition(use_localization),  
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_localization,
                'map_file_name': os.path.join(get_package_share_directory("robot_slam"), 'maps', 'Table2024'),

            }.items()
        ),
        

    ])
