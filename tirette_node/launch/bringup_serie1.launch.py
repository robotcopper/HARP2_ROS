from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Chemin vers le fichier de paramètres du LIDAR
    # lidar_params_file = "/home/harp_2/ros2_ws/src/HARP2_ROS/robot_bringup/params/TminiPro.yaml"
    lidar_params_file = "/home/harp_2/ros2_ws/src/HARP2_ROS/robot_bringup/params/X4.yaml"

    return LaunchDescription([

        # Micro-ROS Agent
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/pico_mobile_base'
            ],
            output='screen'
        ),

        # Lancement du driver YDLIDAR avec les bons paramètres
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch.py'
                )
            ),
            launch_arguments={'params_file': lidar_params_file}.items()
        ),

        # Lancement différé du controller
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('robot_bringup'),
                            'launch',
                            'robot_controller_bringup.launch.py'
                        )
                    ),
                    launch_arguments={
                        'use_rviz': 'False',
                        'use_gazebo': 'False',
                        'use_TopicBasedSystem_hardware_interface': 'True'
                    }.items()
                ),
            ]
        ),

        # TimerAction(
        #     period=12.0,
        #     actions=[
        #         Node(
        #             package='ros2_lx16a_driver',
        #             executable='lx16a_node.py',
        #             name='lx16a_node',
        #             output='screen'
        #         ),
        #     ]
        # ),

        # Lancement différé du nœud tirette
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='tirette_node',
                    executable='serie1',
                    name='tirette_node',
                    output='screen'
                )
            ]
        ),
        
    ])
