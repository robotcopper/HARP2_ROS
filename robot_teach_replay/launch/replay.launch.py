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
    safety_enabled = LaunchConfiguration('safety_enabled')
    use_lidar = LaunchConfiguration('use_lidar')
    bag = LaunchConfiguration('bag')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'launch_on_robot', default_value='False',
            description='Launch micro_ros_agent on /dev/pico_mobile_base'),
        DeclareLaunchArgument(
            'safety_enabled', default_value='True',
            description='Enable collision_monitor safety filter on /cmd_vel'),
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver (required for safety filter)'),
        DeclareLaunchArgument(
            'bag', description='Path to the bag directory to replay (required)'),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_pkg, 'launch', 'safety_layer.launch.py')),
            launch_arguments={'safety_enabled': safety_enabled}.items(),
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--start-paused'],
            output='screen',
        ),

        Node(
            package='robot_teach_replay',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
        ),
    ])
