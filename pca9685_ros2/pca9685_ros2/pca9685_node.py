import rclpy
import board
import busio
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from adafruit_servokit import ServoKit

DEFAULT_STEERING_ANGLE = 90.0
steering_range = [50.0, 130.0]
throttle_values = [-0.2, 0.00, 0.2] # [reverse, neutral, forward]

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
        # self.cmd_vel_sub = self.create_subscription(
        #     Twist,
        #     'cmd_vel',
        #     self.cmd_vel_callback,
        #     10
        # )

        # create a subscriber to the /joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        
    # # callback function that is called when a new message is received on the /cmd_vel topic and prints the message
    # def cmd_vel_callback(self, msg):
    #     # ensure that the throttle value is within the range of the throttle servo
    #     if msg.linear.x < -4:
    #         new_throttle = throttle_values[1] # neutral
    #     else:
    #         new_throttle = msg.linear.x + self.throttle_servo.throttle
    #         if new_throttle > throttle_values[2]:
    #             new_throttle = throttle_values[2]
    #         elif new_throttle < throttle_values[0]:
    #             new_throttle = throttle_values[0]

    #     # ensure that the steering angle is within the range of the steering servo
    #     new_steering_angle = self.steering_servo.angle + msg.angular.z
    #     if new_steering_angle > steering_range[1]:
    #         new_steering_angle = steering_range[1]
    #     elif new_steering_angle < steering_range[0]:
    #         new_steering_angle = steering_range[0]

    #     # set the steering angle and throttle to the values received from the /cmd_vel topic
    #     self.steering_servo.angle = new_steering_angle
    #     self.throttle_servo.throttle = new_throttle

    #     # print the steering angle and throttle values
    #     self.get_logger().info('Steering Angle: {0}'.format(self.steering_servo.angle))
    #     self.get_logger().info('Throttle: {0}'.format(self.throttle_servo.throttle))

# make a new function that is called when a new message is received on the /joy topic
    def joy_callback(self, msg):
        new_steering_angle = DEFAULT_STEERING_ANGLE + msg.axes[3] * -30
        new_throttle = msg.axes[1]
        # scale throttle value to be within the range of the throttle servo
        new_throttle = (new_throttle + 1) / 2 * (throttle_values[2] - throttle_values[0]) + throttle_values[0]

        # set the steering angle and throttle to the values received from the /cmd_vel topic
        self.steering_servo.angle = new_steering_angle
        self.throttle_servo.throttle = new_throttle

        # print the steering angle and throttle values
        self.get_logger().info('Steering Angle: {0}'.format(self.steering_servo.angle))
        self.get_logger().info('Throttle: {0}'.format(self.throttle_servo.throttle))

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