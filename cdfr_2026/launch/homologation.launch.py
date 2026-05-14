import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    use_pull_gpio_reader = LaunchConfiguration('use_pull_gpio_reader')
    use_team_gpio_reader = LaunchConfiguration('use_team_gpio_reader')
    use_calibration_node = LaunchConfiguration('use_calibration_node')

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

    # --- micro_ros_agent: graceful cleanup + sequenced launch --------------
    # 1) SIGTERM (graceful), wait 1s, then SIGKILL fallback. Gives the agent
    #    a chance to close the serial port cleanly, reducing Pico-zombie risk.
    # 2) Wait for /dev/pico_mobile_base to be back (udev re-creates after
    #    USB CDC re-handshake).
    # 3) Optional USB reset via sysfs if `pico_usb_reset.sh` is on PATH.
    # 4) Clear error if port still missing.
    # `pkill -x` matches the binary name exactly (NOT this bash, whose cmd
    # line contains "micro_ros_agent" as text and would be self-killed by -f).
    cleanup_agent = ExecuteProcess(
        condition=IfCondition(launch_on_robot),
        cmd=['bash', '-c',
             'echo "[cleanup] graceful shutdown of any stale micro_ros_agent..."; '
             'pkill -TERM -x micro_ros_agent 2>/dev/null && sleep 1 || true; '
             'pkill -9 -x micro_ros_agent 2>/dev/null || true; '
             'sleep 0.5; '
             'echo "[cleanup] waiting for /dev/pico_mobile_base..."; '
             'for i in $(seq 1 20); do '
             '  if [ -e /dev/pico_mobile_base ]; then '
             '    echo "[cleanup] serial port ready"; '
             '    exit 0; '
             '  fi; '
             '  sleep 0.5; '
             'done; '
             '# Port not back after 10s — try USB reset via sysfs.'
             '# Requires: /etc/sudoers.d/pico_reset with NOPASSWD for this script.'
             'if [ -x /usr/local/bin/pico_usb_reset.sh ]; then '
             '  echo "[cleanup] /dev/pico_mobile_base still missing, trying USB reset..."; '
             '  sudo -n /usr/local/bin/pico_usb_reset.sh || true; '
             '  for i in $(seq 1 10); do '
             '    if [ -e /dev/pico_mobile_base ]; then '
             '      echo "[cleanup] port recovered after USB reset"; '
             '      exit 0; '
             '    fi; '
             '    sleep 0.5; '
             '  done; '
             'fi; '
             'echo "[cleanup] /!\\\\ /dev/pico_mobile_base NOT FOUND - '
             'PICO ZOMBIE, REPLUG IT" >&2; '
             'exit 1'],
        output='screen',
    )

    # sigterm_timeout: give micro_ros_agent 5s to flush & close serial when
    # Ctrl+C'ing, before sending SIGKILL. The agent thus has time to send
    # a final disconnect to the Pico so its USB CDC stays sane.
    micro_ros_agent = ExecuteProcess(
        condition=IfCondition(launch_on_robot),
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', '/dev/pico_mobile_base'],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='2',
    )

    # Sequential: micro_ros_agent only starts AFTER cleanup_agent exits.
    # No Shutdown cascade if the agent later dies — the rest of the launch
    # keeps running. The user can Ctrl+C manually if they want a full shutdown.
    agent_after_cleanup = RegisterEventHandler(
        condition=IfCondition(launch_on_robot),
        event_handler=OnProcessExit(
            target_action=cleanup_agent,
            on_exit=[micro_ros_agent],
        )
    )

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
            'use_pull_gpio_reader', default_value='True',
            description='Launch the cdfr_2026 pull_gpio_reader node (tirette, requires RPi.GPIO)'),
        DeclareLaunchArgument(
            'use_team_gpio_reader', default_value='True',
            description='Launch the cdfr_2026 team_gpio_reader node (team selector, requires RPi.GPIO)'),
        DeclareLaunchArgument(
            'use_calibration_node', default_value='True',
            description='Launch the cdfr_2026 calibration_node (watches /limit_switches, publishes /calibrated)'),
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

        # micro_ros_agent sequence : cleanup -> agent (sequential, no cascade)
        cleanup_agent,
        agent_after_cleanup,

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
            condition=IfCondition(use_pull_gpio_reader),
            package='cdfr_2026',
            executable='pull_gpio_reader',
            name='pull_gpio_reader',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            condition=IfCondition(use_team_gpio_reader),
            package='cdfr_2026',
            executable='team_gpio_reader',
            name='team_gpio_reader',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            condition=IfCondition(use_calibration_node),
            package='cdfr_2026',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='cdfr_2026',
            executable='homologation',
            name='homologation',
            output='screen',
            emulate_tty=True,
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
