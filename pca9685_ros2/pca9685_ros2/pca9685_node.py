import rclpy
import board
import busio
from rclpy.node import Node

from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

default_steering_angle = 90
steering_range = [50, 130]
throttle_values = [0.00, 0.08]

i2c_bus = busio.I2C(board.SCL, board.SDA)

class PCA9685Node(Node):
    def __init__(self):
        super().__init__('pca9685_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        kit = ServoKit(channels=16, i2c=i2c_bus, address=0x40)
        self.steering.Angle = kit.servo[0]
        self.throttle = kit.continuous_servo[1]
        self.steering_angle = default_steering_angle

    def listener_callback(self, msg):
        throttle_value = msg.linear.x
        angle = msg.angular.z
        if throttle_value > 0:
            self.throttle.throttle = throttle_values[1]
        else:
            self.throttle.throttle = throttle_values[0]

        if angle > 0:
            self.steering_angle = max(min(steering_range[1], self.steering_angle + angle), steering_range[0])
        elif angle < 0:
            self.steering_angle = max(min(steering_range[1], self.steering_angle + angle), steering_range[0])

        self.steering_angle.angle = self.steering_angle

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing (linear, angular): ({msg.linear.x}, {msg.angular.z})')


def main(args=None):
    
    rclpy.init(args=args)
    
    pca9685_node = PCA9685Node()
    
    rclpy.spin(pca9685_node)

    pca9685_node.destroy_node()
    
    rclpy.shutdown()



if __name__ == '__main__':
    main()
