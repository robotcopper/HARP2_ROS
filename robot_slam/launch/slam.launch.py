from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
	# Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_localisation = LaunchConfiguration('use_localisation')

    # Specify directory and path to file within package
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
    slam_toolbox_async_params_file_subpath = 'params/mapper_params_online_async.yaml'
    slam_toolbox_localization_params_file_subpath = 'params/mapper_params_localization.yaml'

    if use_localisation:
        slam_params_file=PathJoinSubstitution([slam_toolbox_pkg_dir, slam_toolbox_localization_params_file_subpath])
    else:
        slam_params_file=PathJoinSubstitution([slam_toolbox_pkg_dir, slam_toolbox_async_params_file_subpath])


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
            'slam_params_file',
            default_value=slam_params_file,
            description='Slam parameters file'
        ),
        DeclareLaunchArgument(
            'use_localisation',
            default_value='False',
            description='Whether to use localization mode with slam_toolbox'
        ),


        Node(
            condition=UnlessCondition(use_localisation),
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            output='screen',
            namespace=namespace,
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            condition=IfCondition(use_localisation),
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam',
            output='screen',
            namespace=namespace,
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        ),
        

    ])
