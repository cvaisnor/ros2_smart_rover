# this is a node that reads keyboard input and publishes it to the /cmd_vel topic

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.get_logger().info('Keyboard Controller Node has been started')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = Twist()
        self.settings = termios.tcgetattr(sys.stdin)


    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    

    def listener_callback(self):
        # get the steering angle and throttle values from the /cmd_vel topic
        old_msg = Twist()
        old_msg.linear.x = self.msg.linear.x
        old_msg.angular.z = self.msg.angular.z
        return old_msg
    

    def build_message(self, key, old_msg=None):
        '''This function builds a message based on the key pressed and only changes the steering angle and throttle values that are affected by the key pressed'''
        DEFAULT_STEERING_ANGLE = 90.0
        steering_range = [50.0, 130.0]
        throttle_values = [-0.08, 0.00, 0.08] # [reverse, neutral, forward]
        if old_msg:
            self.msg.linear.x = old_msg.linear.x
            self.msg.angular.z = old_msg.angular.z

        if key == 'w':
            self.msg.linear.x = throttle_values[2] # forward
        elif key == 's':
            self.msg.linear.x = throttle_values[1] # neutral
        elif key == 'a':
            self.msg.angular.z = steering_range[0] # left
        elif key == 'd':
            self.msg.angular.z = steering_range[1] # right
        elif key == 'x':
            self.msg.linear.x = throttle_values[0] # reverse
        elif key == 'q':
            self.msg.linear.x = throttle_values[1]
            self.msg.angular.z = DEFAULT_STEERING_ANGLE
            self.destroy_node()
            rclpy.shutdown()
            sys.exit()


    def timer_callback(self):
        key = self.read_key()
        self.build_message(key)
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Publishing: "{key}"')


def main(args=None):
    rclpy.init(args=args)

    keyboard_controller = KeyboardController()

    rclpy.spin(keyboard_controller)

    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
