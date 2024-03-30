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

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    rviz_config_file_subpath = 'rviz/robot_description_rviz.rviz'

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config_file,],
        output='screen',
        remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose'),
                    ('/robot_description', 'robot_description'),]
    )

    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description=('Top-level namespace.')
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(robot_description_pkg_dir, rviz_config_file_subpath),
            description='Full path to the RVIZ config file to use'
        ),


        rviz2,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action= rviz2,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
            )
        )

    ])