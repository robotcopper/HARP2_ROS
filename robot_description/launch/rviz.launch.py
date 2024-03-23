import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    namespace = LaunchConfiguration('namespace')

    # Get the directory
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    rviz_config_file_subpath = 'rviz/robot_description_rviz.rviz'

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.')
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(robot_description_pkg_dir, rviz_config_file_subpath),
        description='Full path to the RVIZ config file to use'
    )

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=[
            '-d', rviz_config_file,
        ],
        output='screen',
        remappings=[('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
            ('/robot_description', 'robot_description'),
            ]
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        )
    )


    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)

    return ld