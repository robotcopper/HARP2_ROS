import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server', 'amcl']

    # Specify directory and path to file within package
    robot_localization_pkg_dir = get_package_share_directory('robot_localization')
    robot_slam_pkg_dir = get_package_share_directory('robot_slam')
    amcl_params_file_subpath = 'params/amcl.yaml'
    map_file_subpath = 'maps/Table2024.yaml'

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites= {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        },
        convert_types=True
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='namespace'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([robot_slam_pkg_dir, map_file_subpath]),
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([robot_localization_pkg_dir, amcl_params_file_subpath]),
            description='Full path to the ROS2 parameters file to use'
        ),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes}
            ],
        )

    ])