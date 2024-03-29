import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

import xacro

def generate_launch_description():
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.04'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    
    launch_rviz = LaunchConfiguration('launch_rviz')

    robot_sim_pkg_dir = get_package_share_directory('robot_sim')
    wordl_file_subpath = 'world/Table2024.world'

    robot_description_pkg_dir = get_package_share_directory('robot_description')
    urdf_file_subpath = 'urdf/robot.urdf.xacro'
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')
    rviz_config = os.path.join(robot_description_pkg_dir,'rviz','robot_description_rviz.rviz')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gazebo_models_world_path = os.path.join(robot_sim_pkg_dir, 'world')
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_world_path 


    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='False',
        description='True if to launch rviz, false otherwise'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([robot_sim_pkg_dir, wordl_file_subpath ]),
        description='world file'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config]
    )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}],
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'HARP2',
                                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
                    output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnidirectional_controller"],
    )

    omni_base_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[omni_base_controller_spawner]
        )
    )

    rviz_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=omni_base_controller_spawner,
            on_exit=[rviz_node]
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world,
                            }.items()
    )

    return LaunchDescription([
        launch_rviz_arg,
        world_arg,
        joint_state_broadcaster_event_handler,
        omni_base_controller_event_handler,
        robot_state_publisher_node,
        spawn_entity,
        gazebo,
        rviz_event_handler
    ])