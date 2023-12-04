import rclpy
import board
import busio
from rclpy.node import Node

from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

DEFAULT_STEERING_ANGLE = 90.0
steering_range = [50.0, 130.0]
throttle_values = [-0.08, 0.00, 0.08] # [reverse, neutral, forward]

i2c_bus = busio.I2C(board.SCL, board.SDA)

# creating a PCA9685 object node that subscribes to the /cmd_vel topic
class PCA9685Node(Node):

    def __init__(self):
        super().__init__('pca9685_node')
        self.get_logger().info('PCA9685 Node has been started')

        self.kit = ServoKit(channels=16, i2c=i2c_bus)

        # steering servo is connected to channel 0 of the servo driver
        self.steering_servo = self.kit.servo[0]
        self.steering_servo.angle = DEFAULT_STEERING_ANGLE

        # throttle servo is connected to channel 1 of the servo driver
        self.throttle_servo = self.kit.continuous_servo[1]
        self.throttle_servo.throttle = throttle_values[1]

        # create a subscriber to the /cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # note: 
        #     linear.x is throttle
        #     angular.z is steering

        
    # callback function that is called when a new message is received on the /cmd_vel topic and prints the message
    def listener_callback(self, msg):
        # ensure that the throttle value is within the range of the throttle servo
        if msg.linear.x < throttle_values[0]:
            msg.linear.x = throttle_values[0]
        elif msg.linear.x > throttle_values[2]:
            msg.linear.x = throttle_values[2]
        
        # ensure that the steering angle is within the range of the steering servo
        if msg.angular.z == 0:
            msg.angular.z = DEFAULT_STEERING_ANGLE
        if msg.angular.z < steering_range[0]:
            msg.angular.z = steering_range[0]
        elif msg.angular.z > steering_range[1]:
            msg.angular.z = steering_range[1]

        # set the steering angle and throttle to the values received from the /cmd_vel topic
        self.steering_servo.angle = msg.angular.z
        self.throttle_servo.throttle = msg.linear.x

        # print the steering angle and throttle values
        self.get_logger().info(f'Steering angle: {msg.angular.z}, Throttle: {msg.linear.x}')


def main(args=None):

    rclpy.init(args=args)
    pca9685_node = PCA9685Node()
    rclpy.spin(pca9685_node)


    # on shutdown, set steering and throttle to neutral position
    # pca9685_node.steering_servo.angle = DEFAULT_STEERING_ANGLE
    # pca9685_node.throttle_servo.throttle = throttle_values[1]

    pca9685_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()