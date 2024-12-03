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
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = False # Whether to respawn if a node crashes
    log_level = 'info'

    # Specify directory and path to file within package
    robot_nav_pkg_dir = get_package_share_directory('robot_nav')
    nav2_params_file_subpath = 'params/nav2_params.yaml'

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       ]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'default_nav_to_pose_bt_xml' : PathJoinSubstitution([robot_nav_pkg_dir, 'behavior_tree', 'navigate_to_pose_w_replanning_and_recovery.xml']) ,
            'default_nav_through_poses_bt_xml' : PathJoinSubstitution([robot_nav_pkg_dir, 'behavior_tree', 'navigate_through_poses_w_replanning_and_recovery.xml']) ,
        },
        convert_types=True,
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([robot_nav_pkg_dir, nav2_params_file_subpath]),
            description='Full path to the ROS2 parameters file to use',
        ),



        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', '/omnidirectional_controller/cmd_vel_unstamped')], #+ [('cmd_vel', 'cmd_vel_nav')], 
        ),
        # Node(
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=remappings,
        # ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        # Node(
        #     package='nav2_velocity_smoother',
        #     executable='velocity_smoother',
        #     name='velocity_smoother',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     remappings=remappings +
        #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/omnidirectional_controller/cmd_vel_unstamped')],
        # ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}],
        ),

    ])