#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import serial
import threading
import time
import math


class ESP32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge_node')

        # ================= PARAMETERS =================
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)

        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('counts_per_rev', 360.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # ================= SERIAL =================
        self.serial_conn = None
        self.connected = False
        self.connect_serial(serial_port, baud_rate, timeout)

        # ================= ROS PUB/SUB =================
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ================= STATE =================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_enc1 = 0
        self.last_enc2 = 0
        self.last_time = self.get_clock().now()

        # ================= THREAD =================
        self.running = True
        self.read_thread = threading.Thread(
            target=self.read_serial_thread, daemon=True)
        self.read_thread.start()

        self.get_logger().info("ESP32 Bridge Node started")

    # ==================================================
    # SERIAL
    # ==================================================

    def connect_serial(self, port, baud, timeout):
        try:
            self.serial_conn = serial.Serial(
                port=port, baudrate=baud, timeout=timeout)
            time.sleep(2)
            self.connected = True
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.connected = False

    def attempt_reconnect(self):
        self.get_logger().warn("Reconnecting serial...")
        time.sleep(2.0)
        self.connect_serial(
            self.get_parameter('serial_port').value,
            self.get_parameter('baud_rate').value,
            self.get_parameter('timeout').value)

    # ==================================================
    # CMD_VEL
    # ==================================================

    def cmd_vel_callback(self, msg):
        if not self.connected:
            return

        v = msg.linear.x
        w = msg.angular.z

        v_l = v - w * self.wheel_base / 2.0
        v_r = v + w * self.wheel_base / 2.0

        cmd = f"{int(v_l*100)},{int(v_r*100)}\n"

        try:
            self.serial_conn.write(cmd.encode())
        except serial.SerialException:
            self.connected = False
            self.attempt_reconnect()

    # ==================================================
    # SERIAL READ
    # ==================================================

    def read_serial_thread(self):
        while self.running:
            if not self.connected:
                time.sleep(1)
                continue

            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    self.parse_serial_data(line)
            except serial.SerialException:
                self.connected = False
                self.attempt_reconnect()

            time.sleep(0.01)

    def parse_serial_data(self, line):
        try:
            p = line.split(',')
            if len(p) != 8:
                return

            enc1, enc2 = int(p[0]), int(p[1])
            ax, ay, az = float(p[2]), float(p[3]), float(p[4])
            gx, gy, gz = float(p[5]), float(p[6]), float(p[7])

            self.publish_odometry(enc1, enc2)
            self.publish_imu(ax, ay, az, gx, gy, gz)

        except Exception:
            pass

    # ==================================================
    # ODOMETRY
    # ==================================================

    def publish_odometry(self, enc1, enc2):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        d1 = enc1 - self.last_enc1
        d2 = enc2 - self.last_enc2

        d_left = (d1/self.counts_per_rev) * 2*math.pi*self.wheel_radius
        d_right = (d2/self.counts_per_rev) * 2*math.pi*self.wheel_radius

        d_center = (d_left + d_right)/2
        d_theta = (d_right - d_left)/self.wheel_base

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = math.sin(self.theta/2)
        odom.pose.pose.orientation.w = math.cos(self.theta/2)

        odom.twist.twist.linear.x = d_center/dt
        odom.twist.twist.angular.z = d_theta/dt

        # Covariances (important for EKF)
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        self.last_enc1 = enc1
        self.last_enc2 = enc2
        self.last_time = now

    # ==================================================
    # IMU
    # ==================================================

    def publish_imu(self, ax, ay, az, gx, gy, gz):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration_covariance[0] = 0.1
        imu.angular_velocity_covariance[8] = 0.02

        self.imu_pub.publish(imu)

    # ==================================================
    # CLEANUP
    # ==================================================

    def destroy_node(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
