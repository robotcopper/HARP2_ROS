import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import RPi.GPIO as GPIO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
from std_msgs.msg import Float32MultiArray

class GPIOReader(Node):
    def __init__(self):
        super().__init__('gpio_reader')

        self.C = 17
        self.NO = 22
        # self.TEAM = 21

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.C, GPIO.OUT)
        GPIO.setup(self.NO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(self.C, GPIO.HIGH)
        # GPIO.setup(self.TEAM, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.SERVO_PIN = 18
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)  # 50 Hz
        self.servo_pwm.start(0)


        self.publisher_ = self.create_publisher(Bool, 'gpio_state', 10)
        self.publisher_cmdvel = self.create_publisher(Twist, '/omnidirectional_controller/cmd_vel_unstamped', 10)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     qos
        # )

        # self.team = 0
        self.pose_sent = False

        self.last_state = None
        self.obstacle_detected = False

        self.state = 'idle'
        self.state_start_time = None

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('GPIO Reader Node has started.')

        self.timeout_active = False
        self.timeout_deadline = None

    # def scan_callback(self, msg):
        # Vérifie si une mesure est inférieure à 0.2 m
        # for dist in msg.ranges:
        #     if 0.0 < dist < 0.5:
        #         if not self.obstacle_detected:
        #             self.get_logger().warn('Obstacle detected within 30 cm! Stopping motion.')
        #         self.obstacle_detected = True
        #         return
        # self.obstacle_detected = False

    def set_servo_angle(self, angle):
        duty = 2.5 + (angle / 180.0) * 10  # Convertit angle en duty cycle pour servo standard
        self.servo_pwm.ChangeDutyCycle(duty)
        self.get_logger().info(f'Servo set to {angle} degrees (duty cycle: {duty:.2f}%)')

    def read_gpio(self):
        # self.team = GPIO.input(self.TEAM)
        # self.get_logger().info(f'TEAM: {team}.')

        current_state = GPIO.input(self.NO)
        if current_state != self.last_state:
            self.get_logger().info(f'State changed on GPIO {self.NO}: {current_state}')
            msg = Bool()
            msg.data = bool(current_state)
            self.publisher_.publish(msg)

            if current_state == 0:
                self.state = 'forward'
                self.state_start_time = self.get_clock().now()
                self.timeout_active = True
                self.timeout_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=100)
                self.get_logger().info('Countdown started: 100 seconds before permanent stop.')

            self.last_state = current_state

    def get_vector_components(self, distance, angle_deg):
        theta_rad = math.radians(angle_deg)
        x = distance * math.cos(theta_rad)
        y = distance * math.sin(theta_rad)
        return x, y

    def control_loop(self):

        now = self.get_clock().now()

        # Stop everything if timeout has expired
        if self.timeout_active and now > self.timeout_deadline:
            self.get_logger().warn('100-second timeout reached. Robot will no longer move.')
            twist = Twist()
            twist.linear.x = 0.0
            self.publisher_cmdvel.publish(twist)
            return  # Bloque toute commande après timeout

        self.read_gpio()

        twist = Twist()
        now = self.get_clock().now()

        # if self.obstacle_detected:
        #     twist.linear.x = 0.0
        #     self.publisher_cmdvel.publish(twist)
        #     return  # Ne rien faire d'autre tant qu'un obstacle est là

        if self.state == 'forward':
            vel = 0.2
            x, y = self.get_vector_components(vel, -60)  # vitesse de 20 cm/s à 60°
            twist.linear.x = x
            twist.linear.y = y
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 0.2/abs(vel):
                self.state = 'baniere'
                self.state_start_time = now
        
        if self.state == 'baniere':
            elapsed = (now - self.state_start_time).nanoseconds / 1e9

            if elapsed < 2:
                pass  # attendre 3 secondes avant d'agir
            elif elapsed < 5:
                self.set_servo_angle(12)  # action entre 3s et 5s
            else:
                self.state = 'backward'
                self.state_start_time = now




        if self.state == 'backward':
            vel = -0.2
            x, y = self.get_vector_components(vel, -60)  # vitesse de 20 cm/s à 60°
            twist.linear.x = x
            twist.linear.y = y
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 0.5/abs(vel):
                team_choice = 0 #<<<<<<<<<ICI equipe
                if (team_choice == 0):
                    self.state = 'left'
                    self.state_start_time = now
                elif (team_choice == 1):
                    self.state = 'right'
                    self.state_start_time = now

        if self.state == 'left':
            vel = -0.2
            x, y = self.get_vector_components(vel, 20)  # vitesse de 20 cm/s à 60°
            twist.linear.x = x
            twist.linear.y = y
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 0.45/abs(vel):
                self.state = 'to_left_home'
                self.state_start_time = now
        
        if self.state == 'to_left_home':
            vel = -0.2
            x, y = self.get_vector_components(vel, -45)  # vitesse de 20 cm/s à 60°
            twist.linear.x = x
            twist.linear.y = y
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 0.9/abs(vel):
                self.state = 'idle'
                self.state_start_time = now
        
        if self.state == 'right':
            vel = 0.2
            x, y = self.get_vector_components(vel, 20)  # vitesse de 20 cm/s à 60°
            twist.linear.x = x
            twist.linear.y = y
            self.publisher_cmdvel.publish(twist)
            if (now - self.state_start_time).nanoseconds / 1e9 > 0.45/abs(vel):
                self.state = 'to_right_home'
                self.state_start_time = now

        if self.state == 'to_right_home':
                vel = -0.2
                x, y = self.get_vector_components(vel, -75)  # vitesse de 20 cm/s à 60°
                twist.linear.x = x
                twist.linear.y = y
                self.publisher_cmdvel.publish(twist)
                if (now - self.state_start_time).nanoseconds / 1e9 > 0.9/abs(vel):
                    self.state = 'idle'
                    self.state_start_time = now
        

        if self.state == 'idle':
            twist.linear.x = 0.0
            self.publisher_cmdvel.publish(twist)

    def destroy_node(self):
        self.servo_pwm.stop()
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
