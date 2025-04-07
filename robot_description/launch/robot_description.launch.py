import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import xacro



def generate_launch_description():
    
	# Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_joint_state_pub_gui = LaunchConfiguration('use_joint_state_pub_gui')
    robot_description = LaunchConfiguration('robot_description')

    # Specify directory and path to file within package
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    urdf_file_subpath = 'urdf/HARP2_urdf/arm/arm.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(robot_description_pkg_dir, urdf_file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    return LaunchDescription([
        
        # Declare launch arguments
	    DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_raw,
            description='robot_description'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher'
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub',
            default_value='True',
            description='Whether to start the joint state publisher'
        ),
        DeclareLaunchArgument(
            'use_joint_state_pub_gui',
            default_value='False',
            description='Whether to start the joint state publisher GUI'
        ),


		Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description':robot_description,
                    }],
        ),
	    
		Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description,
                    }],
    	),
        
		Node(
        condition=IfCondition(use_joint_state_pub_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        ),

    ])
