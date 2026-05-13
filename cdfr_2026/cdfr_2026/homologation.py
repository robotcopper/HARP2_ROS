import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ANSI escape codes for colored log messages.
class C:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'


# ============================================================================
# DEFAULT SPEEDS — adjust if needed
# ============================================================================
LINEAR_SPEED = 0.2     # m/s for forward/backward/strafe
ANGULAR_SPEED = 0.5    # rad/s for rotations (~28.6 deg/s, so 90 deg = ~3.14 s)


# ============================================================================
# HIGH-LEVEL TRAJECTORY HELPERS
# ----------------------------------------------------------------------------
# Each helper returns a step tuple (vx, vy, vw, duration_s).
# Speeds can be overridden per-call with the optional 'speed' argument.
# ============================================================================
def forward(distance_m, speed=LINEAR_SPEED):
    return (speed, 0.0, 0.0, distance_m / speed)


def backward(distance_m, speed=LINEAR_SPEED):
    return (-speed, 0.0, 0.0, distance_m / speed)


def strafe_left(distance_m, speed=LINEAR_SPEED):
    return (0.0, speed, 0.0, distance_m / speed)


def strafe_right(distance_m, speed=LINEAR_SPEED):
    return (0.0, -speed, 0.0, distance_m / speed)


def rotate_ccw(angle_deg, speed=ANGULAR_SPEED):
    return (0.0, 0.0, speed, math.radians(angle_deg) / speed)


def rotate_cw(angle_deg, speed=ANGULAR_SPEED):
    return (0.0, 0.0, -speed, math.radians(angle_deg) / speed)


def wait(seconds):
    return (0.0, 0.0, 0.0, seconds)


# ============================================================================
# EDIT THIS LIST TO DEFINE YOUR MATCH SEQUENCE
# ----------------------------------------------------------------------------
# The whole list is one match: triggered by the tirette, runs to completion.
# Pause due to obstacle does NOT count towards a step's duration; the step's
# clock only advances when the robot is actually moving.
# Once the sequence is complete (or aborted on safety timeout), further tirette
# pulls are ignored.
# ============================================================================
TRAJECTORY = [
    forward(1.0),
    backward(1.0),
]


class Homologation(Node):
    def __init__(self):
        super().__init__('homologation')

        self.declare_parameter('cmd_vel_topic',
                               '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('gpio_topic', '/gpio_state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.30)
        self.declare_parameter('scan_min_range', 0.19)
        self.declare_parameter('pause_timeout', 90.0)
        self.declare_parameter('match_duration', 100.0)
        self.declare_parameter('trigger_on_low', True)
        self.declare_parameter('control_rate', 20.0)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        gpio_topic = self.get_parameter('gpio_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        self.scan_min_range = float(self.get_parameter('scan_min_range').value)
        self.pause_timeout = float(self.get_parameter('pause_timeout').value)
        self.match_duration = float(self.get_parameter('match_duration').value)
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
        self.obstacle_distance = None
        self.last_tirette_state = None
        self.state = 'idle'
        self.done = False
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.pause_start = None
        self.match_start_time = None
        self.match_window_closed = False

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'{C.BOLD}{C.CYAN}====== HOMOLOGATION READY ======{C.RESET}\n'
            f'  match steps         : {len(TRAJECTORY)}\n'
            f'  match duration      : {self.match_duration}s (hard stop at expiry)\n'
            f'  publishing cmd_vel  : {cmd_topic}\n'
            f'  listening tirette   : {gpio_topic}\n'
            f'  safety distance     : {self.safety_distance}m '
            f'(ignoring returns < {self.scan_min_range}m)\n'
            f'  pause timeout       : {self.pause_timeout}s\n'
            f'  {C.YELLOW}>>> WAITING FOR FIRST /gpio_state MESSAGE <<<{C.RESET}'
        )

    def on_tirette(self, msg: Bool):
        # Only react on state change. /gpio_state arrives at ~20Hz, we don't
        # want to log or evaluate triggers on every steady-state message.
        if self.last_tirette_state == msg.data:
            return

        if msg.data:
            self.get_logger().info(
                f'{C.GREEN}>>> TIRETTE EN PLACE (gpio=HIGH) - '
                f'standby, waiting to be pulled{C.RESET}'
            )
        else:
            self.get_logger().info(
                f'{C.BOLD}{C.MAGENTA}>>> TIRETTE RETIREE (gpio=LOW) - '
                f'MATCH TRIGGER{C.RESET}'
            )
        self.last_tirette_state = msg.data

        triggered = (not msg.data) if self.trigger_on_low else bool(msg.data)
        if not triggered:
            return

        if self.done:
            self.get_logger().warn(
                'match start IGNORED: sequence already completed (or aborted)'
            )
            return

        if self.state != 'idle':
            self.get_logger().warn(
                f'match start IGNORED: still {self.state} at step {self.step_idx}'
            )
            return

        self.state = 'running'
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.match_start_time = time.monotonic()
        self.get_logger().info(
            f'{C.BOLD}{C.GREEN}>>> MATCH START: sequence of {len(TRAJECTORY)} steps, '
            f'{self.match_duration}s on the clock <<<{C.RESET}'
        )

    def on_scan(self, msg: LaserScan):
        # Ignore returns below scan_min_range: those are typically the robot's
        # own structure (lidar is mounted near the center of a 35cm robot).
        d = self.safety_distance
        m = self.scan_min_range
        in_zone = [r for r in msg.ranges if m < r < d]
        if in_zone:
            self.obstacle = True
            self.obstacle_distance = min(in_zone)
        else:
            self.obstacle = False
            self.obstacle_distance = None

    def stop(self):
        self.cmd_pub.publish(Twist())

    def abort_current(self, reason: str):
        self.get_logger().error(
            f'{C.BOLD}{C.RED}aborting match at step {self.step_idx}: {reason}{C.RESET}'
        )
        self.stop()
        self.done = True
        self.state = 'idle'
        self.pause_start = None

    def tick(self):
        # Official match timer: wall-clock since tirette trigger. Runs even
        # after the sequence completed, so we always log when the 100s window
        # actually closes.
        if self.match_start_time is not None and not self.match_window_closed:
            elapsed = time.monotonic() - self.match_start_time
            if elapsed >= self.match_duration:
                if self.state != 'idle':
                    self.get_logger().info(
                        f'{C.BOLD}{C.YELLOW}>>> MATCH TIME UP ({self.match_duration}s) - '
                        f'FULL STOP at step {self.step_idx} <<<{C.RESET}'
                    )
                    self.stop()
                    self.done = True
                    self.state = 'idle'
                    self.pause_start = None
                else:
                    self.get_logger().info(
                        f'{C.BOLD}{C.YELLOW}>>> MATCH TIME UP ({self.match_duration}s) - '
                        f'end of the 100s window <<<{C.RESET}'
                    )
                self.match_window_closed = True
                return

        if self.state == 'idle':
            return

        if self.obstacle:
            if self.state == 'running':
                self.state = 'paused'
                self.pause_start = time.monotonic()
                d = self.obstacle_distance if self.obstacle_distance is not None else 0.0
                self.get_logger().warn(
                    f'{C.BOLD}{C.RED}/!\\ OBSTACLE at {d:.2f}m{C.RESET} '
                    f'(zone {self.scan_min_range:.2f}-{self.safety_distance:.2f}m) '
                    f'-> match {C.BOLD}PAUSED{C.RESET} at step {self.step_idx} '
                    f'(will abort after {self.pause_timeout}s)'
                )
            self.stop()

            paused_for = time.monotonic() - self.pause_start
            if paused_for > self.pause_timeout:
                self.abort_current(
                    f'paused for {paused_for:.1f}s > {self.pause_timeout}s timeout'
                )
            return

        if self.state == 'paused':
            self.state = 'running'
            self.pause_start = None
            self.get_logger().info(
                f'{C.GREEN}>>> obstacle CLEARED -> RESUMING match at step '
                f'{self.step_idx}{C.RESET}'
            )

        vx, vy, vw, duration = TRAJECTORY[self.step_idx]

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vw)
        self.cmd_pub.publish(cmd)

        self.step_elapsed += self.dt

        if self.step_elapsed >= duration:
            self.step_idx += 1
            self.step_elapsed = 0.0

            if self.step_idx >= len(TRAJECTORY):
                self.stop()
                self.get_logger().info(
                    f'{C.BOLD}{C.GREEN}>>> MATCH COMPLETE: '
                    f'{len(TRAJECTORY)} steps done <<<{C.RESET}'
                )
                self.done = True
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
