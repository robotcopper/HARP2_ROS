from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Specify directory and path to file within package
    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_simulation_gazebo_launch_file_subpath = 'launch/ur_sim_control.launch.py'
    rviz_config_file = os.path.join(get_package_share_directory('robot_arm_kdl_trajectory_planner'), "rviz", "view_robot.rviz")

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_simulation_gazebo_pkg_dir, ur_simulation_gazebo_launch_file_subpath)
            ),
            launch_arguments={'ur_type': 'ur5',
                              'launch_rviz': 'False'
                              }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        # Wait 2 seconds for the controller to become available
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_arm_kdl_trajectory_planner',
                    executable='joint_angle_publisher',
                    name='joint_angle_publisher',
                    output='screen',
                ),

                Node(
                    package='robot_arm_kdl_trajectory_planner',
                    executable='cartesian_space_trajectory_node',
                    name='cartesian_space_trajectory_node',
                    output='screen',
                )
            ]
        )
    ])
