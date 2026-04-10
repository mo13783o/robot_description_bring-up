#!/usr/bin/env python3

import math  # FIX: was incorrectly using rclpy.math which does not exist

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import threading
import time


class ESP32BridgeNode(Node):
    """ROS 2 Node for bidirectional communication with ESP32 over serial."""
    self.total_distance = 0.0
    def __init__(self):
        super().__init__('esp32_bridge_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('wheel_base', 0.60)   # metres – must match firmware
        self.declare_parameter('wheel_radius', 0.1) # metres – must match firmware

        serial_port = self.get_parameter('serial_port').value
        baud_rate   = self.get_parameter('baud_rate').value
        timeout     = self.get_parameter('timeout').value

        self.wheel_base   = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # FIX: Use the gear-ratio-corrected ticks/rev for each wheel independently
        self.ticks_per_rev_left  = 875.0
        self.ticks_per_rev_right = 798.0

        # Serial connection
        self.serial_conn = None
        self.connected   = False
        self.connect_serial(serial_port, baud_rate, timeout)

        # FIX: Publish odometry on /odom/raw so the EKF subscribes correctly
        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10)
        # FIX: Publish IMU on /imu/data to match EKF imu0 topic
        self.imu_pub  = self.create_publisher(Imu,      '/imu/data', 10)

        # ROS 2 Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Odometry state
        self.x      = 0.0
        self.y      = 0.0
        self.theta  = 0.0
        self.last_enc_left  = 0
        self.last_enc_right = 0
        self.last_time = self.get_clock().now()

        # Start serial reading thread
        self.running     = True
        self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'ESP32 Bridge Node started on {serial_port} at {baud_rate} baud'
        )

    # ------------------------------------------------------------------
    def connect_serial(self, port, baud, timeout):
        """Establish serial connection with error handling."""
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout
            )
            time.sleep(2)   # Wait for ESP32 to finish reset
            self.connected = True
            self.get_logger().info(f'Connected to ESP32 on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            self.connected = False

    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        """Convert Twist message to left/right wheel speeds and send to ESP32."""
        if not self.connected:
            return

        linear_vel  = msg.linear.x   # m/s
        angular_vel = msg.angular.z  # rad/s

        # Differential drive inverse kinematics
        v_left  = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Scale to the ESP32 command range ±250
        # Adjust this scale factor to match your robot's max velocity
        scale = 100
        speed_left  = int(v_left  * scale)
        speed_right = int(v_right * scale)

        # Clamp to valid range expected by firmware
        speed_left  = max(-250, min(250, speed_left))
        speed_right = max(-250, min(250, speed_right))

        command = f'{speed_left},{speed_right}\n'

        try:
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent to ESP32: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.connected = False
            self.attempt_reconnect()

    # ------------------------------------------------------------------
    def read_serial_thread(self):
        """Continuously read from serial port in background thread."""
        while self.running:
            if not self.connected:
                time.sleep(1.0)
                continue

            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.parse_serial_data(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.connected = False
                self.attempt_reconnect()
            except Exception as e:
                self.get_logger().error(f'Unexpected error reading serial: {e}')

            time.sleep(0.01)  # Prevent CPU spin

    # ------------------------------------------------------------------
    def parse_serial_data(self, line):
        """Parse incoming serial data: 'enc_left,enc_right,ax,ay,az,gx,gy,gz'"""
        try:
            parts = line.split(',')
            if len(parts) != 8:
                self.get_logger().warning(f'Invalid data format: {line}')
                return

            enc_left  = int(parts[0])
            enc_right = int(parts[1])
            ax = float(parts[2])
            ay = float(parts[3])
            az = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])

            self.publish_odometry(enc_left, enc_right)
            self.publish_imu(ax, ay, az, gx, gy, gz)

        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Error parsing data "{line}": {e}')

    # ------------------------------------------------------------------
    def publish_odometry(self, enc_left, enc_right):
        """Compute and publish odometry from encoder values."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # FIX: Reject dt that is zero, negative, or unreasonably large
        # (e.g. after a reconnect or first message)
        if dt <= 0.0 or dt > 1.0:
            self.last_time      = current_time
            self.last_enc_left  = enc_left
            self.last_enc_right = enc_right
            return

        # Encoder delta since last update
        d_enc_left  = enc_left  - self.last_enc_left
        d_enc_right = enc_right - self.last_enc_right

        # FIX: Use per-wheel ticks/rev (gear-ratio corrected)
        d_left  = (d_enc_left  / self.ticks_per_rev_left)  * 2.0 * math.pi * self.wheel_radius
        d_right = (d_enc_right / self.ticks_per_rev_right) * 2.0 * math.pi * self.wheel_radius
        
        # 🔥 DEBUG: encoder movement check
        print(f"[DEBUG] d_left: {d_left:.4f} | d_right: {d_right:.4f}")
        # Differential drive forward kinematics
        d_center = (d_left + d_right) / 2.0
        # FIX: d_theta = (d_right - d_left) / wheel_base
        # Both encoders count positive when moving forward (enforced in firmware).
        # A left-only turn → d_right > d_left → positive (counter-clockwise) theta.
        d_theta  = (d_right - d_left) / self.wheel_base
        self.total_distance += abs(d_center)

        print(f"[DISTANCE] Total: {self.total_distance:.3f} m")
        # Update pose using FIX: math.cos / math.sin (not rclpy.math)
        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)
        self.theta += d_theta

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp    = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation as quaternion from yaw angle
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.total_distance += abs(d_center)

        print(f"[DISTANCE] Total: {self.total_distance:.3f} m")

        # FIX: Provide a real pose covariance instead of all zeros.
        # Diagonal entries (x, y, z, roll, pitch, yaw). Only x and yaw are
        # meaningful for a 2-D differential drive robot.
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 1e-4   # z  (constrained – 2D robot)
        odom.pose.covariance[21] = 1e-4   # roll
        odom.pose.covariance[28] = 1e-4   # pitch
        odom.pose.covariance[35] = 0.03   # yaw

        # Velocity
        odom.twist.twist.linear.x  = d_center / dt
        odom.twist.twist.angular.z = d_theta  / dt

        # FIX: Provide a real twist covariance
        odom.twist.covariance[0]  = 0.01   # vx
        odom.twist.covariance[7]  = 1e-4   # vy (should be ~0)
        odom.twist.covariance[14] = 1e-4   # vz
        odom.twist.covariance[21] = 1e-4   # vroll
        odom.twist.covariance[28] = 1e-4   # vpitch
        odom.twist.covariance[35] = 0.03   # vyaw

        self.odom_pub.publish(odom)

        # Advance state
        self.last_enc_left  = enc_left
        self.last_enc_right = enc_right
        self.last_time      = current_time

    # ------------------------------------------------------------------
    def publish_imu(self, ax, ay, az, gx, gy, gz):
        """Publish IMU data."""
        imu = Imu()
        imu.header.stamp    = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Linear acceleration (m/s²)
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        # Angular velocity (rad/s)
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        # FIX: Provide reasonable covariance values instead of -1.
        # -1 on [0] signals "unknown" to nav_msgs convention and disables
        # the sensor in robot_localization EKF.
        # MPU6050 typical noise figures are used as starting values.
        imu.angular_velocity_covariance[0] = 0.01   # gx variance
        imu.angular_velocity_covariance[4] = 0.01   # gy variance
        imu.angular_velocity_covariance[8] = 0.01   # gz variance

        imu.linear_acceleration_covariance[0] = 0.1   # ax variance
        imu.linear_acceleration_covariance[4] = 0.1   # ay variance
        imu.linear_acceleration_covariance[8] = 0.1   # az variance

        # No absolute orientation is provided by this IMU configuration
        imu.orientation_covariance[0] = -1.0
        print(f"[IMU] gz: {gz:.4f}")
        self.imu_pub.publish(imu)

    # ------------------------------------------------------------------
    def attempt_reconnect(self):
        """Attempt to reconnect to serial port."""
        self.get_logger().warn('Attempting to reconnect...')
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass

        time.sleep(2.0)
        port    = self.get_parameter('serial_port').value
        baud    = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.connect_serial(port, baud, timeout)

    # ------------------------------------------------------------------
    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
