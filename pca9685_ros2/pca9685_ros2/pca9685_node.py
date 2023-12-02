import rclpy
import board
import busio
from rclpy.node import Node

from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

default_steering_angle = 90.0
steering_range = [50.0, 130.0]
throttle_values = [-0.08, 0.00, 0.08] # [reverse, neutral, forward]

i2c_bus = busio.I2C(board.SCL, board.SDA)

# create a class for the PCA9685 node that subscribes to cmd_vel and sets the steering and throttle accordingly
class PCA9685Node(Node):
    def __init__(self):
        super().__init__('pca9685_node')
        self.get_logger().info('Initializing PCA9685 Node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        kit = ServoKit(channels=16, i2c=i2c_bus, address=0x40)
        self.steering_servo = kit.servo[0]
        self.throttle_servo = kit.continuous_servo[1]
        self.steering_servo.angle = default_steering_angle
        self.throttle_servo.throttle = throttle_values[1]

    def listener_callback(self, msg):
        throttle_value = msg.linear.x
        steering_angle = msg.angular.z

        print(f'I heard: (linear.x, angular.z): ({throttle_value}, {steering_angle}))')

        self.steering_servo.angle = steering_angle
        self.throttle_servo.throttle = throttle_value

    # def timer_callback(self):
    #     msg = Twist()
    #     msg.linear.x = throttle_values[1]
    #     msg.angular.z = default_steering_angle
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing (linear.x, angular.z): ({msg.linear.x}, {msg.angular.z})')


def main(args=None):

    rclpy.init(args=args)
    pca9685_node = PCA9685Node()
    rclpy.spin(pca9685_node)

    # on shutdown, set steering and throttle to neutral position
    pca9685_node.steering_servo.angle = default_steering_angle
    pca9685_node.throttle_servo.throttle = throttle_values[1]

    pca9685_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
