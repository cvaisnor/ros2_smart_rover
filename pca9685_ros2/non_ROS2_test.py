import time
import busio
import board
import sys
import select
import tty
import termios
from adafruit_servokit import ServoKit

print("Initializing i2c bus")
# PCA9685 is on bus 7 at address 0x40
i2c_bus = busio.I2C(board.SCL, board.SDA)

print("Initializing servo kit")
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x40)

print('Setting steering and throttle channels')
steering = kit.servo[0]
steering_range = [50, 130] # [max left, max right]
steering.angle = 90 # set steering to neutral position

# channel 1 needs to be set for PWM DC motor control
throttle = kit.continuous_servo[1]
throttle_values = [-0.08, 0.00, 0.08] # [reverse, neutral, forward]

def read_keyboard_input():
    # if user presses a key, return the key
    # otherwise return None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            ch = sys.stdin.read(1)
        else:
            ch = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    print('Starting main loop')
    # wait for the user to press c
    # ensures the ESC calibration is done before starting the loop
    if input("Press c to continue") == 'c':
        
        # set throttle to neutral position
        throttle.throttle = throttle_values[1]
        # set steering to neutral position
        steering.angle = 90
        
        print("Starting loop")
        while True:
            key = read_keyboard_input()

            if key == 'w':
                print("Throttle Forward")
                throttle.throttle = throttle_values[2]

            if key == 's':
                print("Throttle Neutral")
                throttle.throttle = throttle_values[1]

            if key == 'x':
                print("Throttle Reverse")
                throttle.throttle = throttle_values[0]

            if key == 'a':
                if steering.angle > steering_range[0]:
                    # print("Steering Left by 1 degree")
                    steering.angle = steering.angle - 5
                    # print("Steering angle: ", steering.angle)
                else:
                    print("Left steering limit reached")

            if key == 'd':
                if steering.angle < steering_range[1]:
                    # print("Steering Right by 1 degree")
                    steering.angle = steering.angle + 5
                    # print("Steering angle: ", steering.angle)
                else:
                    print("Right steering limit reached")

            # if q is pressed, break the loop
            if key == 'q':
                print("Returning to neutral positions")
                steering.angle = 90
                throttle.throttle = 0
                break

if __name__ == "__main__":
    main()