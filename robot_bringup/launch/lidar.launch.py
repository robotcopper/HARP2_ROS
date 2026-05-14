from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    scan_topic = LaunchConfiguration('scan_topic')

    default_params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'params', 'X4.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='True',
            description='Launch the YDLIDAR driver node. Required for the safety check to receive /scan.'),
        DeclareLaunchArgument(
            'lidar_params_file', default_value=default_params,
            description='Path to lidar parameters YAML (X4.yaml or TminiPro.yaml).'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value='ydlidarx4_laser_frame',
            description='frame_id override applied on top of the YAML. Set so it matches the laser_joint child link in your URDF, regardless of which lidar model is mounted.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Output topic to remap /scan to (e.g. /ydlidarx4/scan or /tminipro/scan).'),

        # Driver launched directly (NOT via ydlidar_launch.py from the submodule)
        # so we skip its bogus static_transform_publisher to base_link->laser_frame.
        # The robot's TF tree is fully provided by robot_state_publisher + URDF.
        LifecycleNode(
            condition=IfCondition(use_lidar),
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            output='screen',
            emulate_tty=True,
            parameters=[
                lidar_params_file,
                {'frame_id': lidar_frame_id},
            ],
            remappings=[('/scan', scan_topic)],
        ),
    ])
