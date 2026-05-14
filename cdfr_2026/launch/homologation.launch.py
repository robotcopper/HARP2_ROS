import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression


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
    lidar_model = LaunchConfiguration('lidar_model')
    use_gpio_reader = LaunchConfiguration('use_gpio_reader')

    # /ydlidarx4/scan for X4, /tminipro/scan otherwise.
    scan_topic = PythonExpression([
        "'/ydlidarx4/scan' if '", lidar_model, "' == 'X4' else '/tminipro/scan'"
    ])
    safety_distance = LaunchConfiguration('safety_distance')
    scan_min_range = LaunchConfiguration('scan_min_range')
    safety_cone_deg = LaunchConfiguration('safety_cone_deg')
    lidar_yaw_offset = LaunchConfiguration('lidar_yaw_offset')
    pause_timeout = LaunchConfiguration('pause_timeout')
    match_duration = LaunchConfiguration('match_duration')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'launch_on_robot', default_value='False',
            description='Launch micro_ros_agent on /dev/pico_mobile_base'),
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver'),
        DeclareLaunchArgument(
            'lidar_model', default_value='TminiPro',
            description='Lidar model: X4 or TminiPro. Selects the YAML in robot_bringup/params/ and the scan topic name (/ydlidarx4/scan or /tminipro/scan).'),
        DeclareLaunchArgument(
            'use_gpio_reader', default_value='True',
            description='Launch the cdfr_2026 gpio_reader node (requires RPi.GPIO on a Raspberry Pi)'),
        DeclareLaunchArgument(
            'safety_distance', default_value='0.5',
            description='Min distance (m) to any scan point before pausing'),
        DeclareLaunchArgument(
            'scan_min_range', default_value='0.16',
            description='Ignore scan returns below this (robot self-detection)'),
        DeclareLaunchArgument(
            'safety_cone_deg', default_value='60.0',
            description='Total angle of safety cone around motion direction (deg)'),
        DeclareLaunchArgument(
            'lidar_yaw_offset', default_value='0.523599',
            description='Lidar yaw relative to base_link (rad, from URDF laser_joint)'),
        DeclareLaunchArgument(
            'pause_timeout', default_value='90.0',
            description='Seconds of continuous pause before aborting current trajectory'),
        DeclareLaunchArgument(
            'match_duration', default_value='100.0',
            description='Total match duration in seconds (wall-clock from tirette pull)'),

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
            launch_arguments={
                'use_lidar': use_lidar,
                'lidar_params_file': [
                    robot_bringup_pkg + '/params/',
                    lidar_model,
                    '.yaml',
                ],
                'scan_topic': scan_topic,
            }.items(),
        ),

        Node(
            condition=IfCondition(use_gpio_reader),
            package='cdfr_2026',
            executable='gpio_reader',
            name='gpio_reader',
            output='screen',
        ),

        Node(
            package='cdfr_2026',
            executable='homologation',
            name='homologation',
            output='screen',
            parameters=[{
                'safety_distance': safety_distance,
                'scan_min_range': scan_min_range,
                'safety_cone_deg': safety_cone_deg,
                'lidar_yaw_offset': lidar_yaw_offset,
                'pause_timeout': pause_timeout,
                'match_duration': match_duration,
                'scan_topic': scan_topic,
            }],
        ),
    ])
