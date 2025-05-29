from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Chemins vers les fichiers nécessaires
    lidar_params_file = "/home/harp_2/ros2_ws/src/HARP2/robot_bringup/params/TminiPro.yaml"
    
    # Suppose que robot_bringup et ydlidar_ros2_driver sont bien trouvés via ament
    # Sinon, utilise directement les chemins absolus pour les fichiers .launch.py

    return LaunchDescription([

        # Launch MicroROS
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/pico_mobile_base'
            ],
            output='screen'
        ),

        # Launch YDLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ydlidar_ros2_driver'),
                    'launch',
                    'ydlidar_launch.py'
                )
            ]),
            launch_arguments={'params_file': lidar_params_file}.items()
        ),

        # Launch robot_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robot_bringup'),
                    'launch',
                    'robot_controller_bringup.launch.py'
                )
            ]),
            launch_arguments={
                'use_rviz': 'False',
                'use_gazebo': 'False',
                'use_TopicBasedSystem_hardware_interface': 'True'
            }.items()
        ),

        # Run tirette_node
        Node(
            package='tirette_node',
            executable='serie1',
            name='serie1',
            output='screen'
        )
    ])
