import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

class GPIOReader(Node):
    def __init__(self):
        super().__init__('gpio_reader')

        # Config GPIO
        self.C=17
        # self.NC = 27
        self.NO = 22

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.C, GPIO.OUT)
        # GPIO.setup(self.NO, GPIO.IN)
        GPIO.setup(self.NO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Important !

        # Set output states
        GPIO.output(self.C, GPIO.HIGH)

        self.publisher_ = self.create_publisher(Bool, 'gpio_state', 10)
        self.last_state = None
        self.timer = self.create_timer(0.01, self.read_gpio)  # 10 ms

        self.get_logger().info('GPIO Reader Node has started.')

    def read_gpio(self):
        current_state = GPIO.input(self.NO)
        if current_state != self.last_state:
            msg = Bool()
            msg.data = bool(current_state)
            self.publisher_.publish(msg)
            self.get_logger().info(f'State changed on GPIO {self.NO}: {current_state}')
            self.last_state = current_state

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
