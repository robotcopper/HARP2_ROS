import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ============================================================================
# EDIT THIS LIST TO DEFINE YOUR TRAJECTORIES
# ----------------------------------------------------------------------------
# Each trajectory is a list of steps. Each step is (vx, vy, vw, duration_s).
#   vx, vy : m/s in base_link frame (vx = forward, vy = left)
#   vw     : rad/s (positive = counter-clockwise)
#   duration_s : how long to maintain that velocity
#
# Pause due to obstacle does NOT count towards the duration: the step's clock
# only advances when the robot is actually moving.
# ============================================================================
TRAJECTORIES = [
    # Trajectory 0 : forward 1.0 m
    [
        (0.2, 0.0, 0.0, 5.0),
    ],

    # Trajectory 1 : rotate ~90 deg then forward 0.5 m
    [
        (0.0, 0.0, 0.5, math.pi / 2 / 0.5),
        (0.2, 0.0, 0.0, 2.5),
    ],

    # Trajectory 2 : square (forward, left, backward, right) of 0.5 m each
    [
        (0.2, 0.0, 0.0, 2.5),
        (0.0, 0.2, 0.0, 2.5),
        (-0.2, 0.0, 0.0, 2.5),
        (0.0, -0.2, 0.0, 2.5),
    ],
]


class Homologation(Node):
    def __init__(self):
        super().__init__('homologation')

        self.declare_parameter('cmd_vel_topic',
                               '/omnidirectional_controller/cmd_vel_safety_unstamped')
        self.declare_parameter('gpio_topic', '/gpio_state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.30)
        self.declare_parameter('pause_timeout', 90.0)
        self.declare_parameter('trigger_on_low', True)
        self.declare_parameter('control_rate', 20.0)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        gpio_topic = self.get_parameter('gpio_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        self.pause_timeout = float(self.get_parameter('pause_timeout').value)
        self.trigger_on_low = bool(self.get_parameter('trigger_on_low').value)
        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / rate

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Bool, gpio_topic, self.on_tirette, 10)

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, scan_topic, self.on_scan, scan_qos)

        self.obstacle = False
        self.state = 'idle'
        self.traj_idx = 0
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.pause_start = None

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'homologation ready: {len(TRAJECTORIES)} trajectories loaded, '
            f'publishing to {cmd_topic}, listening to {gpio_topic}'
        )

    def on_tirette(self, msg: Bool):
        triggered = (msg.data is False) if self.trigger_on_low else (msg.data is True)
        if not triggered:
            return

        if self.state != 'idle':
            self.get_logger().warn(
                f'tirette ignored: still {self.state} on trajectory {self.traj_idx}'
            )
            return

        if self.traj_idx >= len(TRAJECTORIES):
            self.get_logger().warn(
                f'tirette ignored: all {len(TRAJECTORIES)} trajectories already played'
            )
            return

        self.state = 'running'
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.get_logger().info(
            f'tirette -> starting trajectory {self.traj_idx} '
            f'({len(TRAJECTORIES[self.traj_idx])} steps)'
        )

    def on_scan(self, msg: LaserScan):
        d = self.safety_distance
        self.obstacle = any(0.0 < r < d for r in msg.ranges)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def abort_current(self, reason: str):
        self.get_logger().error(
            f'aborting trajectory {self.traj_idx} at step {self.step_idx}: {reason}'
        )
        self.stop()
        self.traj_idx += 1
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.state = 'idle'
        self.pause_start = None

    def tick(self):
        if self.state == 'idle':
            return

        if self.obstacle:
            if self.state == 'running':
                self.state = 'paused'
                self.pause_start = self.get_clock().now()
                self.get_logger().warn(
                    f'obstacle within {self.safety_distance}m -> pausing'
                )
            self.stop()

            paused_for = (self.get_clock().now() - self.pause_start).nanoseconds / 1e9
            if paused_for > self.pause_timeout:
                self.abort_current(
                    f'paused for {paused_for:.1f}s > {self.pause_timeout}s timeout'
                )
            return

        if self.state == 'paused':
            self.state = 'running'
            self.pause_start = None
            self.get_logger().info('clear -> resuming')

        traj = TRAJECTORIES[self.traj_idx]
        vx, vy, vw, duration = traj[self.step_idx]

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vw)
        self.cmd_pub.publish(cmd)

        self.step_elapsed += self.dt

        if self.step_elapsed >= duration:
            self.step_idx += 1
            self.step_elapsed = 0.0

            if self.step_idx >= len(traj):
                self.stop()
                self.get_logger().info(
                    f'trajectory {self.traj_idx} done. '
                    f'Next available: {self.traj_idx + 1}/{len(TRAJECTORIES)}'
                )
                self.traj_idx += 1
                self.step_idx = 0
                self.state = 'idle'


def main():
    rclpy.init()
    node = Homologation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
