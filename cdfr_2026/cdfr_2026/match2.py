import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
LINEAR_SPEED = 0.4     # m/s for forward/backward/strafe
ANGULAR_SPEED = 0.8    # rad/s for rotations (~28.6 deg/s, so 90 deg = ~3.14 s)


# ============================================================================
# HIGH-LEVEL TRAJECTORY HELPERS
# ----------------------------------------------------------------------------
# Each helper returns a step tuple (vx, vy, vw, target, kind):
#   - kind='linear'  -> target is meters (euclidean displacement from start)
#   - kind='angular' -> target is radians (accumulated |dyaw| from start)
#   - kind='wait'    -> target is seconds (time-based, only mode that uses it)
# Speeds can be overridden per-call with the optional 'speed' argument.
# Termination is purely odom-based for linear/angular steps.
# ============================================================================
def forward(distance_m, speed=LINEAR_SPEED):
    return (speed, 0.0, 0.0, distance_m, 'linear')


def backward(distance_m, speed=LINEAR_SPEED):
    return (-speed, 0.0, 0.0, distance_m, 'linear')


def strafe_left(distance_m, speed=LINEAR_SPEED):
    return (0.0, speed, 0.0, distance_m, 'linear')


def strafe_right(distance_m, speed=LINEAR_SPEED):
    return (0.0, -speed, 0.0, distance_m, 'linear')


# Translation along any heading, expressed in the robot's local frame.
# angle_deg is measured CCW from the robot's +X axis (forward):
#   0    -> forward                  45  -> forward-left
#   90   -> strafe_left              135 -> backward-left
#   180  -> backward                -45  -> forward-right
#  -90   -> strafe_right            -135 -> backward-right
# Magnitude hypot(vx,vy) is always `speed`, so cruise speed and the trapezoidal
# ramp behave identically regardless of heading. `distance_m` is the euclidean
# displacement along that heading, which is what closed-loop progress measures.
def diagonal(distance_m, angle_deg, speed=LINEAR_SPEED):
    a = math.radians(angle_deg)
    return (speed * math.cos(a), speed * math.sin(a), 0.0,
            distance_m, 'linear')


def rotate_ccw(angle_deg, speed=ANGULAR_SPEED):
    return (0.0, 0.0, speed, math.radians(angle_deg), 'angular')


def rotate_cw(angle_deg, speed=ANGULAR_SPEED):
    return (0.0, 0.0, -speed, math.radians(angle_deg), 'angular')


def wait(seconds):
    return (0.0, 0.0, 0.0, seconds, 'wait')


def mirror_x(trajectory):
    """Mirror a trajectory across the robot's X axis at start position.

    Y-axis lateral motion and rotation directions are flipped; forward/backward
    and waits are unchanged. This matches the CDFR rule that the blue side is
    the geometric mirror of the yellow side across the table's long axis.
    """
    return [(vx, -vy, -vw, target, kind)
            for (vx, vy, vw, target, kind) in trajectory]


def rotate_local(step, delta_deg):
    """Rotate (vx, vy) of a linear step by delta_deg in the robot's local frame.

    Used when one side's rotation step is overridden: the robot's final
    orientation diverges from the mirror by `delta_deg`, so any subsequent
    local-frame translation must be rotated by the same delta to keep its
    world-frame direction unchanged.
    """
    vx, vy, vw, target, kind = step
    if kind != 'linear':
        return step
    a = math.radians(delta_deg)
    c, s = math.cos(a), math.sin(a)
    return (c * vx - s * vy, s * vx + c * vy, vw, target, kind)


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
    rotate_ccw(180),
    strafe_left(0.12),
    backward(0.75),
    forward(0.10),
    strafe_right(0.30),
    backward(1.1),
    forward(0.10),
    strafe_left(0.40),
    rotate_ccw(240),
    diagonal(0.14, -30),
    diagonal(0.1, 60),
    diagonal(0.56, 145),
    diagonal(1.8, -106),
]    

# Explicit blue trajectory. Mostly the X-axis mirror of YELLOW_TRAJECTORY,
# except index 8 (the second rotation) is rotate_cw(120) instead of the
# mirrored rotate_cw(240) because the robot arm is not symmetric.
BLUE_TRAJECTORY = [
    rotate_cw(180),         # mirror of rotate_ccw(180)
    strafe_right(0.12),     # mirror of strafe_left(0.12)
    backward(0.75),         # unchanged by mirror (vy=0)
    forward(0.10),          # unchanged by mirror (vy=0)
    strafe_left(0.30),      # mirror of strafe_right(0.30)
    backward(1.1),          # unchanged by mirror (vy=0)
    forward(0.10),          # unchanged by mirror (vy=0)
    strafe_right(0.40),     # mirror of strafe_left(0.40)
    rotate_cw(120),         # OVERRIDE: asymmetric arm (mirror would be rotate_cw(240))
    # The 4 diagonals below are the X-mirror of YELLOW then rotated by -120
    # deg in the local frame, to compensate the 120 deg orientation gap
    # introduced by the rotation override above, so they end up pointing in
    # the X-mirrored world directions of the yellow ones.
    diagonal(0.14, -90),    # mirror -> 30; -120 -> -90 (strafe_right)
    diagonal(0.1, -180),    # mirror -> -60; -120 -> -180 (backward)
    diagonal(0.56, 95),     # mirror -> -145; -120 -> -265 -> +95
    diagonal(1.8, -14),     # mirror -> 106; -120 -> -14
]


class Match2(Node):
    def __init__(self):
        super().__init__('match2')

        self.declare_parameter('cmd_vel_topic',
                               '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic',
                               '/omnidirectional_controller/odom')
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
        # Closed-loop overshoot compensation: each motion step's target is
        # reduced by this amount, so the robot stops earlier and coasts
        # roughly onto the requested displacement.
        self.declare_parameter('linear_overshoot_m', 0.0)
        self.declare_parameter('rotation_overshoot_deg', 0.0)
        # Trapezoidal velocity profile (accel up, cruise, decel down).
        # Larger values = snappier moves but more end-of-step rocking;
        # smaller = smoother but slower steps.
        self.declare_parameter('linear_accel', 0.4)   # m/s^2
        self.declare_parameter('angular_accel', 1.0)  # rad/s^2
        # team_a_is_yellow: maps team_gpio_reader's "team A" (switch CLOSED,
        # gpio HIGH, msg.data=True) to the YELLOW trajectory. Flip to False
        # if the wiring/convention is reversed.
        self.declare_parameter('team_a_is_yellow', True)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
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
        self.linear_overshoot_m = float(
            self.get_parameter('linear_overshoot_m').value)
        self.rotation_overshoot_rad = math.radians(
            float(self.get_parameter('rotation_overshoot_deg').value))
        self.linear_accel = max(
            float(self.get_parameter('linear_accel').value), 1e-3)
        self.angular_accel = max(
            float(self.get_parameter('angular_accel').value), 1e-3)
        self.team_a_is_yellow = bool(self.get_parameter('team_a_is_yellow').value)
        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / rate

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Bool, gpio_topic, self.on_tirette, 10)
        self.create_subscription(Bool, team_topic, self.on_team, 10)

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)

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
        self.step_elapsed = 0.0      # used for wait() steps only
        # Closed-loop step tracking. step_kind in {'linear','angular','wait'};
        # step_start_pose snapshots odom at the first running tick of a step.
        # Linear progress is euclidean displacement from step_start_pose;
        # angular progress accumulates the abs of inter-tick yaw deltas to
        # survive crossings of the ±pi wrap (a single-shot diff would saturate
        # at pi and never trigger termination on a 180+ deg rotation).
        self.odom_pose = None         # (x, y, yaw) from /odom
        self.step_kind = None
        self.step_target = 0.0
        self.step_start_pose = None
        self.step_traveled_angle = 0.0
        self.last_odom_yaw = None
        self.step_alpha = 0.0   # trapezoidal velocity scaling [0,1]
        self.pause_start = None
        self.match_start_time = None
        self.match_window_closed = False

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'{C.BOLD}{C.CYAN}====== MATCH 2 READY ======{C.RESET}\n'
            f'  yellow steps        : {len(YELLOW_TRAJECTORY)}\n'
            f'  blue steps          : {len(BLUE_TRAJECTORY)}\n'
            f'  match duration      : {self.match_duration}s (hard stop at expiry)\n'
            f'  publishing cmd_vel  : {cmd_topic}\n'
            f'  listening odom      : {odom_topic} (closed-loop on displacement)\n'
            f'  listening tirette   : {gpio_topic}\n'
            f'  listening team      : {team_topic} '
            f'(team A = {"YELLOW" if self.team_a_is_yellow else "BLUE"})\n'
            f'  listening calibrated: {calibrated_topic}\n'
            f'  safety distance     : {self.safety_distance}m '
            f'(ignoring returns < {self.scan_min_range}m)\n'
            f'  pause timeout       : {self.pause_timeout}s\n'
            f'  overshoot comp.     : linear -{self.linear_overshoot_m:.3f}m, '
            f'rotation -{math.degrees(self.rotation_overshoot_rad):.1f}deg\n'
            f'  {C.YELLOW}>>> WAITING FOR CALIBRATION + TEAM + ODOM + TIRETTE <<<{C.RESET}'
        )

    def on_odom(self, msg: Odometry):
        # Extract yaw from quaternion. We don't need a full tf2 stack here:
        # the controller publishes a 2D pose with the planar yaw encoded in
        # (z, w) primarily, but using the full formula is safe and cheap.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_pose = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          yaw)

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

        if self.odom_pose is None:
            self.get_logger().warn(
                f'{C.YELLOW}match start IGNORED: no odometry received yet '
                f'(closed-loop needs /odom){C.RESET}'
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
        self.step_kind = None
        self.step_target = 0.0
        self.step_start_pose = None
        self.step_traveled_angle = 0.0
        self.last_odom_yaw = None
        self.step_alpha = 0.0
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

        vx, vy, _, _, _ = self.trajectory[self.step_idx]
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
            # Restart the ramp from rest: the robot has been stopped during
            # the pause, so we re-accelerate from 0 instead of stepping back
            # to the pre-pause velocity.
            self.step_alpha = 0.0
            self.get_logger().info(
                f'{C.GREEN}>>> obstacle CLEARED -> RESUMING match at step '
                f'{self.step_idx}{C.RESET}'
            )

        vx, vy, vw, target, kind = self.trajectory[self.step_idx]

        # First running tick of this step: apply overshoot margin and snapshot.
        if self.step_kind is None:
            self.step_kind = kind
            if kind == 'linear':
                self.step_target = max(target - self.linear_overshoot_m, 0.0)
            elif kind == 'angular':
                self.step_target = max(
                    target - self.rotation_overshoot_rad, 0.0)
            else:
                self.step_target = target
            self.step_start_pose = self.odom_pose
            self.step_traveled_angle = 0.0
            self.last_odom_yaw = self.odom_pose[2]
            self.step_elapsed = 0.0
            self.step_alpha = 0.0
            unit = {'linear': 'm', 'angular': 'rad', 'wait': 's'}[kind]
            self.get_logger().info(
                f'{C.CYAN}step {self.step_idx + 1}/{len(self.trajectory)} '
                f'[{kind}] target={self.step_target:.3f}{unit}{C.RESET}'
            )

        # Progress: odom-based for motion, time-based for wait. Must be
        # computed BEFORE publishing so the ramp can see the latest state
        # and trigger deceleration at the right moment.
        if self.step_kind == 'wait':
            self.step_elapsed += self.dt
            progress = self.step_elapsed
        elif self.step_kind == 'linear':
            x0, y0, _ = self.step_start_pose
            x, y, _ = self.odom_pose
            progress = math.hypot(x - x0, y - y0)
        else:  # angular: integrate inter-tick |dyaw| to survive pi wrap
            yaw = self.odom_pose[2]
            dyaw = math.atan2(math.sin(yaw - self.last_odom_yaw),
                              math.cos(yaw - self.last_odom_yaw))
            self.step_traveled_angle += abs(dyaw)
            self.last_odom_yaw = yaw
            progress = self.step_traveled_angle

        # Trapezoidal velocity profile: ramp alpha in [0,1] up to cruise,
        # then ramp down as remaining distance shrinks below the stopping
        # distance v^2 / (2 a). For wait steps alpha is irrelevant — we
        # publish zero velocity.
        if self.step_kind == 'wait':
            self.cmd_pub.publish(Twist())
        else:
            if self.step_kind == 'linear':
                cruise = math.hypot(vx, vy)
                accel = self.linear_accel
            else:  # angular
                cruise = abs(vw)
                accel = self.angular_accel

            if cruise > 1e-6:
                v_actual = self.step_alpha * cruise
                d_to_stop = (v_actual * v_actual) / (2.0 * accel)
                remaining = max(self.step_target - progress, 0.0)
                d_alpha = (accel * self.dt) / cruise
                if remaining <= d_to_stop:
                    self.step_alpha = max(self.step_alpha - d_alpha, 0.0)
                else:
                    self.step_alpha = min(self.step_alpha + d_alpha, 1.0)
            else:
                self.step_alpha = 0.0

            cmd = Twist()
            cmd.linear.x = float(vx) * self.step_alpha
            cmd.linear.y = float(vy) * self.step_alpha
            cmd.angular.z = float(vw) * self.step_alpha
            self.cmd_pub.publish(cmd)

        if progress >= self.step_target:
            self.get_logger().info(
                f'{C.GREEN}step {self.step_idx + 1} done '
                f'(reached {progress:.3f} >= {self.step_target:.3f}){C.RESET}'
            )
            self.step_idx += 1
            self.step_elapsed = 0.0
            self.step_kind = None
            self.step_target = 0.0
            self.step_start_pose = None
            self.step_traveled_angle = 0.0
            self.last_odom_yaw = None
            self.step_alpha = 0.0

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
    node = Match2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
