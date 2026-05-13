import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_controller_pkg = get_package_share_directory('robot_controller')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')

    xacro_file = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = Command([
        'xacro ', xacro_file,
        ' use_controller:=True',
        ' use_TopicBasedSystem_hardware_interface:=True',
    ])

    namespace = LaunchConfiguration('namespace')
    launch_on_robot = LaunchConfiguration('launch_on_robot')
    use_lidar = LaunchConfiguration('use_lidar')
    safety_distance = LaunchConfiguration('safety_distance')
    pause_timeout = LaunchConfiguration('pause_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'launch_on_robot', default_value='False',
            description='Launch micro_ros_agent on /dev/pico_mobile_base'),
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver'),
        DeclareLaunchArgument(
            'safety_distance', default_value='0.30',
            description='Min distance (m) to any scan point before pausing'),
        DeclareLaunchArgument(
            'pause_timeout', default_value='90.0',
            description='Seconds of continuous pause before aborting current trajectory'),

        ExecuteProcess(
            condition=IfCondition(launch_on_robot),
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'serial', '--dev', '/dev/pico_mobile_base'],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_pkg, 'launch', 'robot_description.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'False',
                'use_robot_state_pub': 'True',
                'use_joint_state_pub': 'True',
                'use_joint_state_pub_gui': 'False',
                'robot_description': robot_description_raw,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_controller_pkg, 'launch', 'controller.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_TopicBasedSystem_hardware_interface': 'True',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={'use_lidar': use_lidar}.items(),
        ),

        Node(
            package='cdfr_2026',
            executable='homologation',
            name='homologation',
            output='screen',
            parameters=[{
                'safety_distance': safety_distance,
                'pause_timeout': pause_timeout,
            }],
        ),
    ])
