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


def mirror_x(trajectory):
    """Mirror a trajectory across the robot's X axis at start position.

    Y-axis lateral motion and rotation directions are flipped; forward/backward
    and waits are unchanged. This matches the CDFR rule that the blue side is
    the geometric mirror of the yellow side across the table's long axis.
    """
    return [(vx, -vy, -vw, dt) for (vx, vy, vw, dt) in trajectory]


# ============================================================================
# EDIT THIS LIST TO DEFINE THE YELLOW TEAM MATCH SEQUENCE
# ----------------------------------------------------------------------------
# BLUE_TRAJECTORY is automatically derived from YELLOW_TRAJECTORY by X-axis
# mirroring (see mirror_x). Override BLUE_TRAJECTORY explicitly only if the
# blue script needs to differ from the mirror image of the yellow one.
#
# The whole list is one match: triggered by the tirette, runs to completion.
# Pause due to obstacle does NOT count towards a step's duration; the step's
# clock only advances when the robot is actually moving.
# Once the sequence is complete (or aborted on safety timeout), further tirette
# pulls are ignored.
# ============================================================================
YELLOW_TRAJECTORY = [
    forward(1.0),
    backward(1.0),
]

BLUE_TRAJECTORY = mirror_x(YELLOW_TRAJECTORY)


class Match1(Node):
    def __init__(self):
        super().__init__('match1')

        self.declare_parameter('cmd_vel_topic',
                               '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('gpio_topic', '/pull_gpio_state')
        self.declare_parameter('team_topic', '/team_gpio_state')
        self.declare_parameter('calibrated_topic', '/calibrated')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('safety_distance', 0.22)
        self.declare_parameter('scan_min_range', 0.16)
        self.declare_parameter('safety_cone_deg', 60.0)
        self.declare_parameter('lidar_yaw_offset', 0.523599)
        self.declare_parameter('pause_timeout', 90.0)
        self.declare_parameter('match_duration', 100.0)
        self.declare_parameter('trigger_on_low', True)
        self.declare_parameter('control_rate', 20.0)
        # team_a_is_yellow: maps team_gpio_reader's "team A" (switch CLOSED,
        # gpio HIGH, msg.data=True) to the YELLOW trajectory. Flip to False
        # if the wiring/convention is reversed.
        self.declare_parameter('team_a_is_yellow', True)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        gpio_topic = self.get_parameter('gpio_topic').value
        team_topic = self.get_parameter('team_topic').value
        calibrated_topic = self.get_parameter('calibrated_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.safety_distance = float(self.get_parameter('safety_distance').value)
        self.scan_min_range = float(self.get_parameter('scan_min_range').value)
        # Parameter is total cone angle; internal value is half-angle for math.
        self.safety_cone_rad = math.radians(
            float(self.get_parameter('safety_cone_deg').value) / 2.0)
        self.lidar_yaw_offset = float(self.get_parameter('lidar_yaw_offset').value)
        self.pause_timeout = float(self.get_parameter('pause_timeout').value)
        self.match_duration = float(self.get_parameter('match_duration').value)
        self.trigger_on_low = bool(self.get_parameter('trigger_on_low').value)
        self.team_a_is_yellow = bool(self.get_parameter('team_a_is_yellow').value)
        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / rate

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Bool, gpio_topic, self.on_tirette, 10)
        self.create_subscription(Bool, team_topic, self.on_team, 10)

        # /calibrated is published with TRANSIENT_LOCAL by calibration_node,
        # so we subscribe with the same durability to get the last value.
        calibrated_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, calibrated_topic, self.on_calibrated, calibrated_qos)

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, scan_topic, self.on_scan, scan_qos)

        self.obstacle = False
        self.obstacle_distance = None
        self.last_tirette_state = None
        self.calibrated = False
        self.last_calibrated_logged = None
        self.team_state = None  # latest Bool from team_gpio_reader
        self.last_team_logged = None
        self.trajectory = None  # latched at match start
        self.team_name = None   # latched at match start
        self.state = 'idle'
        self.done = False
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.pause_start = None
        self.match_start_time = None
        self.match_window_closed = False

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'{C.BOLD}{C.CYAN}====== MATCH 1 READY ======{C.RESET}\n'
            f'  yellow steps        : {len(YELLOW_TRAJECTORY)}\n'
            f'  blue steps          : {len(BLUE_TRAJECTORY)}\n'
            f'  match duration      : {self.match_duration}s (hard stop at expiry)\n'
            f'  publishing cmd_vel  : {cmd_topic}\n'
            f'  listening tirette   : {gpio_topic}\n'
            f'  listening team      : {team_topic} '
            f'(team A = {"YELLOW" if self.team_a_is_yellow else "BLUE"})\n'
            f'  listening calibrated: {calibrated_topic}\n'
            f'  safety distance     : {self.safety_distance}m '
            f'(ignoring returns < {self.scan_min_range}m)\n'
            f'  pause timeout       : {self.pause_timeout}s\n'
            f'  {C.YELLOW}>>> WAITING FOR CALIBRATION + TEAM + TIRETTE <<<{C.RESET}'
        )

    def on_calibrated(self, msg: Bool):
        self.calibrated = bool(msg.data)
        if self.last_calibrated_logged != self.calibrated:
            if self.calibrated:
                self.get_logger().info(
                    f'{C.BOLD}{C.GREEN}>>> /calibrated = True - '
                    f'pre-match calibration DONE, tirette enabled{C.RESET}'
                )
            else:
                if self.last_calibrated_logged is True:
                    self.get_logger().warn(
                        f'{C.YELLOW}>>> /calibrated = False - tirette pull '
                        f'will be ignored until calibration completes{C.RESET}'
                    )
            self.last_calibrated_logged = self.calibrated

    def on_team(self, msg: Bool):
        self.team_state = bool(msg.data)
        if self.last_team_logged != self.team_state:
            is_yellow = (self.team_state == self.team_a_is_yellow)
            color = 'YELLOW' if is_yellow else 'BLUE'
            color_ansi = C.YELLOW if is_yellow else C.BLUE
            bar = '=' * 60
            self.get_logger().info(
                f'\n{C.BOLD}{color_ansi}{bar}\n'
                f'====   TEAM SELECTED: {color:^7}   '
                f'(team_gpio={self.team_state})   ====\n'
                f'{bar}{C.RESET}'
            )
            self.last_team_logged = self.team_state

    def on_tirette(self, msg: Bool):
        # Only react on state change. /gpio_state arrives at ~20Hz, we don't
        # want to log or evaluate triggers on every steady-state message.
        if self.last_tirette_state == msg.data:
            return

        is_initial = self.last_tirette_state is None
        self.last_tirette_state = msg.data

        if msg.data:
            self.get_logger().info(
                f'{C.GREEN}>>> TIRETTE EN PLACE (gpio=HIGH) - '
                f'standby, waiting to be pulled{C.RESET}'
            )
        else:
            if is_initial:
                self.get_logger().warn(
                    f'{C.YELLOW}>>> TIRETTE NOT IN PLACE at boot (gpio=LOW) - '
                    f'IGNORED, plug the tirette in and pull it to trigger the match{C.RESET}'
                )
            else:
                self.get_logger().info(
                    f'{C.BOLD}{C.MAGENTA}>>> TIRETTE RETIREE (gpio=LOW) - '
                    f'MATCH TRIGGER{C.RESET}'
                )

        # First /pull_gpio_state message reports the boot-time state. Even if
        # it's LOW, do NOT trigger the match: we only trigger on an actual
        # high->low transition (user pulling the tirette).
        if is_initial:
            return

        triggered = (not msg.data) if self.trigger_on_low else bool(msg.data)
        if not triggered:
            return

        if not self.calibrated:
            self.get_logger().warn(
                f'{C.YELLOW}match start IGNORED: /calibrated is False, '
                f'wait for the pre-match calibration to complete first{C.RESET}'
            )
            return

        if self.team_state is None:
            self.get_logger().warn(
                f'{C.YELLOW}match start IGNORED: team selector state unknown '
                f'(no /team_gpio_state received yet){C.RESET}'
            )
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

        # Latch team choice and trajectory at trigger time.
        is_yellow = (self.team_state == self.team_a_is_yellow)
        if is_yellow:
            self.trajectory = YELLOW_TRAJECTORY
            self.team_name = 'YELLOW'
            color_ansi = C.YELLOW
        else:
            self.trajectory = BLUE_TRAJECTORY
            self.team_name = 'BLUE'
            color_ansi = C.BLUE

        self.state = 'running'
        self.step_idx = 0
        self.step_elapsed = 0.0
        self.match_start_time = time.monotonic()
        self.get_logger().info(
            f'{C.BOLD}{color_ansi}>>> MATCH START [{self.team_name}]: '
            f'sequence of {len(self.trajectory)} steps, '
            f'{self.match_duration}s on the clock <<<{C.RESET}'
        )

    def on_scan(self, msg: LaserScan):
        # Directional safety: only flag points within a cone around the current
        # commanded motion direction. Pure rotations and wait steps don't
        # trigger. scan_min_range filters out the robot's own structure.
        if self.state == 'idle' or self.trajectory is None \
                or self.step_idx >= len(self.trajectory):
            self.obstacle = False
            self.obstacle_distance = None
            return

        vx, vy, _, _ = self.trajectory[self.step_idx]
        if math.hypot(vx, vy) < 0.01:
            self.obstacle = False
            self.obstacle_distance = None
            return

        # Motion direction in base_link, then converted to lidar frame.
        motion_dir_lidar = math.atan2(vy, vx) - self.lidar_yaw_offset
        motion_dir_lidar = math.atan2(math.sin(motion_dir_lidar),
                                      math.cos(motion_dir_lidar))

        d = self.safety_distance
        m = self.scan_min_range
        cone = self.safety_cone_rad

        nearest = None
        for i, r in enumerate(msg.ranges):
            if not (m < r < d):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            diff = angle - motion_dir_lidar
            diff = math.atan2(math.sin(diff), math.cos(diff))
            if abs(diff) <= cone and (nearest is None or r < nearest):
                nearest = r

        if nearest is not None:
            self.obstacle = True
            self.obstacle_distance = nearest
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
                        f'{C.BOLD}{C.YELLOW}>>> MATCH TIME UP at {elapsed:.2f}s '
                        f'(limit {self.match_duration}s) - '
                        f'FULL STOP at step {self.step_idx} <<<{C.RESET}'
                    )
                    self.stop()
                    self.done = True
                    self.state = 'idle'
                    self.pause_start = None
                else:
                    self.get_logger().info(
                        f'{C.BOLD}{C.YELLOW}>>> MATCH TIME UP at {elapsed:.2f}s '
                        f'(limit {self.match_duration}s) - '
                        f'end of the match window <<<{C.RESET}'
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

        vx, vy, vw, duration = self.trajectory[self.step_idx]

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vw)
        self.cmd_pub.publish(cmd)

        self.step_elapsed += self.dt

        if self.step_elapsed >= duration:
            self.step_idx += 1
            self.step_elapsed = 0.0

            if self.step_idx >= len(self.trajectory):
                self.stop()
                match_elapsed = (time.monotonic() - self.match_start_time
                                 if self.match_start_time is not None else 0.0)
                self.get_logger().info(
                    f'{C.BOLD}{C.GREEN}>>> MATCH COMPLETE [{self.team_name}]: '
                    f'{len(self.trajectory)} steps done in {match_elapsed:.2f}s '
                    f'(of {self.match_duration}s window) <<<{C.RESET}'
                )
                self.done = True
                self.state = 'idle'


def main():
    rclpy.init()
    node = Match1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
