#!/usr/bin/env python3
"""
Simple Ackerman teleop:
  w = forward
  s = reverse
  a = steer left (only works while moving)
  d = steer right (only works while moving)
  space = stop
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class AckermannTeleop(Node):
    SPEED_STEP  = 0.05
    STEER_STEP  = 0.05
    MAX_SPEED   = 0.5
    MAX_STEER   = 0.524   # 30 degrees

    def __init__(self):
        super().__init__('ackermann_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.get_logger().info(
            '\nAckermann Teleop\n'
            'w/s = speed up / slow down\n'
            'a/d = steer left / right\n'
            'space = stop\n'
            'q = quit\n'
        )

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        while True:
            key = self.get_key()
            cmd = Twist()

            if key == 'w':
                self.speed = min(self.speed + self.SPEED_STEP, self.MAX_SPEED)
            elif key == 's':
                self.speed = max(self.speed - self.SPEED_STEP, -self.MAX_SPEED)
            elif key == 'a':
                self.steer = min(self.steer + self.STEER_STEP, self.MAX_STEER)
            elif key == 'd':
                self.steer = max(self.steer - self.STEER_STEP, -self.MAX_STEER)
            elif key == ' ':
                self.speed = 0.0
                self.steer = 0.0
            elif key == 'q':
                break

            cmd.linear.x  = self.speed
            cmd.angular.z = self.steer
            self.pub.publish(cmd)

            print(f'\rspeed={self.speed:.2f} m/s  '
                  f'steer={self.steer:.2f} rad  ', end='')


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()