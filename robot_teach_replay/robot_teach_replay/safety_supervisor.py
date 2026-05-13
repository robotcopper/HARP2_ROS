import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CollisionMonitorState
from rosbag2_interfaces.srv import Pause, Resume


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')

        self.declare_parameter('player_node', '/rosbag2_player')
        self.declare_parameter('state_topic', '/collision_monitor_state')
        self.declare_parameter('start_paused', True)

        player = self.get_parameter('player_node').value
        state_topic = self.get_parameter('state_topic').value
        self.is_paused = bool(self.get_parameter('start_paused').value)

        self.pause_cli = self.create_client(Pause, f'{player}/pause')
        self.resume_cli = self.create_client(Resume, f'{player}/resume')

        self.create_subscription(
            CollisionMonitorState, state_topic, self.on_state, 10)

        self.get_logger().info(
            f'waiting for {player}/pause and {player}/resume...')
        self.connected = False
        self.create_timer(0.5, self.check_services)

    def check_services(self):
        if self.connected:
            return
        if self.pause_cli.service_is_ready() and self.resume_cli.service_is_ready():
            self.connected = True
            self.get_logger().info('rosbag2 player services connected')
            if not self.is_paused:
                self.resume_cli.call_async(Resume.Request())

    def on_state(self, msg):
        if not self.connected:
            return

        danger = msg.action_type != CollisionMonitorState.DO_NOTHING

        if danger and not self.is_paused:
            self.get_logger().warn(
                f'obstacle (polygon={msg.polygon_name}, '
                f'action={msg.action_type}) -> pausing bag')
            self.pause_cli.call_async(Pause.Request())
            self.is_paused = True
        elif not danger and self.is_paused:
            self.get_logger().info('clear -> resuming bag')
            self.resume_cli.call_async(Resume.Request())
            self.is_paused = False


def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
