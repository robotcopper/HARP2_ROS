import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


class GpioReader(Node):
    def __init__(self):
        super().__init__('gpio_reader')

        self.declare_parameter('com_pin', 17)
        self.declare_parameter('no_pin', 22)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('topic', '/gpio_state')

        self.com_pin = int(self.get_parameter('com_pin').value)
        self.no_pin = int(self.get_parameter('no_pin').value)
        rate = float(self.get_parameter('publish_rate').value)
        topic = self.get_parameter('topic').value

        if GPIO is None:
            self.get_logger().fatal(
                'RPi.GPIO not installed. This node must run on a Raspberry Pi '
                'with `pip install RPi.GPIO`.'
            )
            sys.exit(1)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.com_pin, GPIO.OUT)
        GPIO.setup(self.no_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(self.com_pin, GPIO.HIGH)

        self.publisher_ = self.create_publisher(Bool, topic, 10)

        self.last_state = None
        self.create_timer(1.0 / rate, self.read_and_publish)

        self.get_logger().info(
            f'gpio_reader started: BCM com={self.com_pin}, no={self.no_pin}, '
            f'rate={rate}Hz, topic={topic}'
        )

    def read_and_publish(self):
        current = GPIO.input(self.no_pin)
        msg = Bool()
        msg.data = bool(current)
        self.publisher_.publish(msg)

        if current != self.last_state:
            label = 'CONNECTED' if current else 'DISCONNECTED'
            self.get_logger().info(f'tirette {label} (GPIO {self.no_pin} = {current})')
            self.last_state = current

    def destroy_node(self):
        if GPIO is not None:
            GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = GpioReader()
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
