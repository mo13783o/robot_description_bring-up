#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # تغيير نوع الرسالة ليتوافق مع EKF و Navigation

def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.read(1)
    return ''

class RobotTeleop(Node):
    def __init__(self):
        super().__init__('robot_keyboard_teleop')

        # النشر على /cmd_vel ليتوافق مع الـ EKF والـ SLAM Toolbox
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # إعدادات السرعة (بالمتر في الثانية والراديان)
        self.linear_vel = 0.2    # m/s
        self.angular_vel = 0.5   # rad/s
        self.speed_step = 0.05

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.print_help()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def print_help(self):
        self.get_logger().info(
            "\nControl Your Robot (Topic: cmd_vel):\n"
            "  w : Forward\n"
            "  s : Backward\n"
            "  a : Turn Left\n"
            "  d : Turn Right\n"
            "  x : Full Stop\n"
            "  q : Increase Speed\n"
            "  e : Decrease Speed\n"
            f"Current Settings: Linear={self.linear_vel}m/s, Angular={self.angular_vel}rad/s\n"
        )

    def update_speed(self, key: str):
        if key == 'w':
            self.target_linear = self.linear_vel
            self.target_angular = 0.0
        elif key == 's':
            self.target_linear = -self.linear_vel
            self.target_angular = 0.0
        elif key == 'a':
            self.target_linear = 0.0
            self.target_angular = self.angular_vel
        elif key == 'd':
            self.target_linear = 0.0
            self.target_angular = -self.angular_vel
        elif key == 'x':
            self.target_linear = 0.0
            self.target_angular = 0.0
        elif key == 'q':
            self.linear_vel += self.speed_step
            self.get_logger().info(f"Speed increased: {self.linear_vel}")
        elif key == 'e':
            self.linear_vel = max(0.0, self.linear_vel - self.speed_step)
            self.get_logger().info(f"Speed decreased: {self.linear_vel}")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    node = RobotTeleop()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key()
            if key:
                if key == '\x03': break
                node.update_speed(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()