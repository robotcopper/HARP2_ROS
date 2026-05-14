import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, UInt8MultiArray
from geometry_msgs.msg import Twist


# ANSI escape codes for colored log messages.
class C:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'


# State machine
S_IDLE = 'idle'
S_HOMING_RIGHT = 'homing_right'
S_BACKUP_RIGHT = 'backup_right'
S_ROTATE = 'rotate'
S_HOMING_LEFT = 'homing_left'
S_BACKUP_LEFT = 'backup_left'
S_FINAL_ROTATE = 'final_rotate'
S_DONE = 'done'
S_FAILED = 'failed'


class CalibrationNode(Node):
    """Active pre-match calibration / homing routine.

    Drives the robot through a fixed sequence to find a known zero:
      1. Translate along the rear-right wheel direction until switches
         {switches_right} are both pressed.
      2. Back up the same direction by {right_backup_m} meters.
      3. Rotate by {rotation_angle_deg} (CCW if positive).
      4. Translate along the rear-left wheel direction until switches
         {switches_left} are both pressed.
      5. Back up the same direction by {left_backup_m} meters.

    Publishes /calibrated (Bool, latched) — False initially, True when the
    sequence completes successfully. While the sequence is running, the node
    publishes Twist commands on cmd_vel_topic. It stops publishing once done.
    """

    def __init__(self):
        super().__init__('calibration_node')

        # --- Topics
        self.declare_parameter('cmd_vel_topic',
                               '/omnidirectional_controller/cmd_vel_unstamped')
        self.declare_parameter('limit_switches_topic', '/limit_switches')
        self.declare_parameter('calibrated_topic', '/calibrated')

        # --- Geometry & motion
        self.declare_parameter('translation_speed', 0.05)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('right_dir_deg', -120.0)
        self.declare_parameter('left_dir_deg', 120.0)
        self.declare_parameter('right_backup_m', 0.216325)
        self.declare_parameter('left_backup_m', 0.141325)
        self.declare_parameter('rotation_angle_deg', -30.0)
        self.declare_parameter('final_rotation_angle_deg', 60.0)

        # --- Limit switch indices
        self.declare_parameter('switches_right', [2, 3])
        self.declare_parameter('switches_left', [0, 1])

        # --- Timing / safety
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('homing_timeout', 30.0)
        self.declare_parameter('auto_start', True)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        limits_topic = self.get_parameter('limit_switches_topic').value
        calib_topic = self.get_parameter('calibrated_topic').value

        self.v_t = float(self.get_parameter('translation_speed').value)
        self.v_r = float(self.get_parameter('rotation_speed').value)
        self.right_dir_rad = math.radians(
            float(self.get_parameter('right_dir_deg').value))
        self.left_dir_rad = math.radians(
            float(self.get_parameter('left_dir_deg').value))
        self.right_backup_m = float(self.get_parameter('right_backup_m').value)
        self.left_backup_m = float(self.get_parameter('left_backup_m').value)
        self.rotation_angle_rad = math.radians(
            float(self.get_parameter('rotation_angle_deg').value))
        self.final_rotation_angle_rad = math.radians(
            float(self.get_parameter('final_rotation_angle_deg').value))
        self.switches_right = list(self.get_parameter('switches_right').value)
        self.switches_left = list(self.get_parameter('switches_left').value)
        rate = float(self.get_parameter('control_rate').value)
        self.dt = 1.0 / rate
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.homing_timeout = float(self.get_parameter('homing_timeout').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)

        # --- IO
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.calib_pub = self.create_publisher(Bool, calib_topic, latched_qos)
        self.create_subscription(
            UInt8MultiArray, limits_topic, self.on_limits, 10)

        # --- State
        self.switches = []
        self.state = S_IDLE
        self.phase_start_time = None
        self.phase_duration = None
        self.node_start_time = time.monotonic()

        # Initial /calibrated = False (latched, late subscribers see it)
        self.publish_calibrated(False)

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'{C.BOLD}{C.CYAN}====== CALIBRATION READY ======{C.RESET}\n'
            f'  cmd_vel out         : {cmd_topic}\n'
            f'  limit switches in   : {limits_topic}\n'
            f'  calibrated out      : {calib_topic} (latched)\n'
            f'  right dir / backup  : {math.degrees(self.right_dir_rad):+.1f}° '
            f'/ {self.right_backup_m*100:.3f} cm (switches {self.switches_right})\n'
            f'  left dir / backup   : {math.degrees(self.left_dir_rad):+.1f}° '
            f'/ {self.left_backup_m*100:.3f} cm (switches {self.switches_left})\n'
            f'  mid rotation        : {math.degrees(self.rotation_angle_rad):+.1f}°\n'
            f'  final rotation      : {math.degrees(self.final_rotation_angle_rad):+.1f}°\n'
            f'  speeds              : v_t = {self.v_t} m/s, v_r = {self.v_r} rad/s\n'
            f'  homing timeout      : {self.homing_timeout}s\n'
            f'  auto_start          : {self.auto_start} '
            f'(starting in {self.start_delay}s if True)'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_limits(self, msg: UInt8MultiArray):
        self.switches = list(msg.data)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def all_pressed(self, indices):
        if not self.switches:
            return False
        for i in indices:
            if i >= len(self.switches) or not self.switches[i]:
                return False
        return True

    def translation_twist(self, direction_rad):
        twist = Twist()
        twist.linear.x = self.v_t * math.cos(direction_rad)
        twist.linear.y = self.v_t * math.sin(direction_rad)
        return twist

    def rotation_twist(self, angular_speed):
        twist = Twist()
        twist.angular.z = angular_speed
        return twist

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_calibrated(self, calibrated: bool):
        msg = Bool()
        msg.data = calibrated
        self.calib_pub.publish(msg)

    def transition(self, new_state: str, msg: str, color: str = C.MAGENTA):
        self.get_logger().info(
            f'{C.BOLD}{color}[CAL] -> {new_state.upper()}: {msg}{C.RESET}'
        )
        self.state = new_state
        self.phase_start_time = time.monotonic()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()

        if self.state == S_IDLE:
            if self.auto_start and (now - self.node_start_time) >= self.start_delay:
                self.transition(
                    S_HOMING_RIGHT,
                    f'translating at {math.degrees(self.right_dir_rad):+.1f}° '
                    f'until switches {self.switches_right} pressed',
                    color=C.CYAN,
                )
            return

        if self.state == S_DONE or self.state == S_FAILED:
            return

        # Homing timeout guard for active-contact phases.
        if self.state in (S_HOMING_RIGHT, S_HOMING_LEFT):
            elapsed = now - self.phase_start_time
            if elapsed > self.homing_timeout:
                self.stop()
                self.transition(
                    S_FAILED,
                    f'homing timeout after {elapsed:.1f}s '
                    f'(switches not reached)',
                    color=C.RED,
                )
                return

        if self.state == S_HOMING_RIGHT:
            self.cmd_pub.publish(self.translation_twist(self.right_dir_rad))
            if self.all_pressed(self.switches_right):
                self.stop()
                self.phase_duration = self.right_backup_m / self.v_t
                self.transition(
                    S_BACKUP_RIGHT,
                    f'backing up {self.right_backup_m*100:.3f} cm '
                    f'over {self.phase_duration:.2f}s',
                    color=C.CYAN,
                )
            return

        if self.state == S_BACKUP_RIGHT:
            self.cmd_pub.publish(
                self.translation_twist(self.right_dir_rad + math.pi))
            if (now - self.phase_start_time) >= self.phase_duration:
                self.stop()
                self.phase_duration = abs(self.rotation_angle_rad) / self.v_r
                self.transition(
                    S_ROTATE,
                    f'rotating {math.degrees(self.rotation_angle_rad):+.1f}° '
                    f'over {self.phase_duration:.2f}s',
                    color=C.CYAN,
                )
            return

        if self.state == S_ROTATE:
            self.cmd_pub.publish(
                self.rotation_twist(math.copysign(self.v_r, self.rotation_angle_rad)))
            if (now - self.phase_start_time) >= self.phase_duration:
                self.stop()
                self.transition(
                    S_HOMING_LEFT,
                    f'translating at {math.degrees(self.left_dir_rad):+.1f}° '
                    f'until switches {self.switches_left} pressed',
                    color=C.CYAN,
                )
            return

        if self.state == S_HOMING_LEFT:
            self.cmd_pub.publish(self.translation_twist(self.left_dir_rad))
            if self.all_pressed(self.switches_left):
                self.stop()
                self.phase_duration = self.left_backup_m / self.v_t
                self.transition(
                    S_BACKUP_LEFT,
                    f'backing up {self.left_backup_m*100:.3f} cm '
                    f'over {self.phase_duration:.2f}s',
                    color=C.CYAN,
                )
            return

        if self.state == S_BACKUP_LEFT:
            self.cmd_pub.publish(
                self.translation_twist(self.left_dir_rad + math.pi))
            if (now - self.phase_start_time) >= self.phase_duration:
                self.stop()
                self.phase_duration = abs(self.final_rotation_angle_rad) / self.v_r
                self.transition(
                    S_FINAL_ROTATE,
                    f'final rotation {math.degrees(self.final_rotation_angle_rad):+.1f}° '
                    f'over {self.phase_duration:.2f}s',
                    color=C.CYAN,
                )
            return

        if self.state == S_FINAL_ROTATE:
            self.cmd_pub.publish(self.rotation_twist(
                math.copysign(self.v_r, self.final_rotation_angle_rad)))
            if (now - self.phase_start_time) >= self.phase_duration:
                self.stop()
                self.publish_calibrated(True)
                self.transition(
                    S_DONE,
                    f'CALIBRATED ✅ - zero established, ready for tirette',
                    color=C.GREEN,
                )
            return


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
