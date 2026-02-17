#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def get_key():
    """قراءة مفتاح واحد من الكيبورد بدون Enter (non-blocking)."""
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)
    if dr:
        return sys.stdin.read(1)
    return ''


class WheelTeleop(Node):
    def __init__(self):
        super().__init__('wheel_keyboard_teleop')

        # الناشر على /wheel_cmd (String: "R L")
        self.pub = self.create_publisher(String, '/wheel_cmd', 10)

        # إعدادات السرعة
        self.base_speed = 25    # سرعة أساسية كبداية
        self.speed_step = 5
        self.max_speed = 250
        self.min_speed = -250

        # أوامر حالية
        self.r = 0
        self.l = 0

        # تايمر للنشر الدوري (كل 0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.print_help()

    def print_help(self):
        self.get_logger().info(
            "\nKeyboard teleop for hoverboard (topic /wheel_cmd):\n"
            "  w : forward\n"
            "  s : backward\n"
            "  a : turn left\n"
            "  d : turn right\n"
            "  x : stop (0,0)\n"
            "  q : increase base speed\n"
            "  e : decrease base speed\n"
            "  CTRL+C : exit\n"
            f"Base speed = {self.base_speed}\n"
        )

    def update_speed(self, key: str):
        # تعديل (r,l) بناءً على الزر
        if key == 'w':
            self.r = self.base_speed
            self.l = self.base_speed
        elif key == 's':
            self.r = -self.base_speed
            self.l = -self.base_speed
        elif key == 'a':
            # لف لليسار
            self.r = self.base_speed
            self.l = -self.base_speed
        elif key == 'd':
            # لف لليمين
            self.r = -self.base_speed
            self.l = self.base_speed
        elif key == 'x':
            self.r = 0
            self.l = 0
        elif key == 'q':
            self.base_speed = min(self.base_speed + self.speed_step, self.max_speed)
            self.get_logger().info(f"Base speed increased to {self.base_speed}")
        elif key == 'e':
            self.base_speed = max(self.base_speed - self.speed_step, 0)
            self.get_logger().info(f"Base speed decreased to {self.base_speed}")

        # حدود الأمان
        self.r = max(self.min_speed, min(self.max_speed, self.r))
        self.l = max(self.min_speed, min(self.max_speed, self.l))

        self.get_logger().info(f"Command -> R: {self.r}, L: {self.l}")

    def timer_callback(self):
        # نشر الأوامر الحالية على /wheel_cmd
        msg = String()
        msg.data = f"{self.r} {self.l}"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # حفظ إعدادات الطرفية
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    node = WheelTeleop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key()
            if key:
                if key == '\x03':  # CTRL+C
                    break
                node.update_speed(key)
    finally:
        # رجّع إعدادات الطرفية كما كانت
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
