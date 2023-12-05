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
    

    def build_message(self, key):
        '''This function builds a message based on the key pressed and only changes the steering angle and throttle values that are affected by the key pressed'''

        # after each message is published, the steering angle and throttle values are reset to their default values 
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        if key == 'w':
            self.msg.linear.x = 0.08 # increase throttle
        elif key == 's':
            self.msg.linear.x = -5.0 # stop
        elif key == 'a':
            self.msg.angular.z = -10.0 # turn left x degrees
        elif key == 'd':
            self.msg.angular.z = 10.0 # right turn x degrees
        elif key == 'x':
            self.msg.linear.x = -0.02 # decrease throttle
        elif key == 'q':
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
