import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import RPi.GPIO as GPIO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class GPIOReader(Node):
    def __init__(self):
        super().__init__('gpio_reader')

        self.C = 17
        self.NO = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.C, GPIO.OUT)
        GPIO.setup(self.NO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(self.C, GPIO.HIGH)

        self.publisher_ = self.create_publisher(Bool, 'gpio_state', 10)
        self.publisher_cmdvel = self.create_publisher(Twist, '/omnidirectional_controller/cmd_vel_unstamped', 10)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10
        # )

        self.last_state = None
        self.obstacle_detected = False

        self.state = 'idle'
        self.state_start_time = None

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('GPIO Reader Node has started.')

    def scan_callback(self, msg):
        # Vérifie si une mesure est inférieure à 0.3 m
        for dist in msg.ranges:
            if 0.0 < dist < 0.5:
                if not self.obstacle_detected:
                    self.get_logger().warn('Obstacle detected within 30 cm! Stopping motion.')
                self.obstacle_detected = True
                return
        self.obstacle_detected = False

    def read_gpio(self):
        current_state = GPIO.input(self.NO)
        if current_state != self.last_state:
            self.get_logger().info(f'State changed on GPIO {self.NO}: {current_state}')
            msg = Bool()
            msg.data = bool(current_state)
            self.publisher_.publish(msg)

            if current_state == 0:
                self.state = 'forward'
                self.state_start_time = self.get_clock().now()

            self.last_state = current_state

    def control_loop(self):
        self.read_gpio()

        twist = Twist()
        now = self.get_clock().now()

        if self.obstacle_detected:
            twist.linear.x = 0.0
            self.publisher_cmdvel.publish(twist)
            return  # Ne rien faire d'autre tant qu'un obstacle est là

        if self.state == 'forward':
            twist.linear.x = 0.15
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 4.0:
                self.state = 'backward'
                self.state_start_time = now

        elif self.state == 'backward':
            twist.linear.x = -0.15
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 4.0:
                self.state = 'idle'
                twist.linear.x = 0.0
                self.publisher_cmdvel.publish(twist)

        else:
            twist.linear.x = 0.0
            self.publisher_cmdvel.publish(twist)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPIOReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
