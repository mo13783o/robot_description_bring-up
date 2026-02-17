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


class ESP32BridgeNode(Node):
    """ROS 2 Node for bidirectional communication with ESP32 over serial."""
    
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('wheel_base', 0.2)  # meters
        self.declare_parameter('wheel_radius', 0.05)  # meters
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Serial connection
        self.serial_conn = None
        self.connected = False
        self.connect_serial(serial_port, baud_rate, timeout)
        
        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        # ROS 2 Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc1 = 0
        self.last_enc2 = 0
        self.last_time = self.get_clock().now()
        
        # Start serial reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info(f'ESP32 Bridge Node started on {serial_port} at {baud_rate} baud')
    
    def connect_serial(self, port, baud, timeout):
        """Establish serial connection with error handling."""
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.connected = True
            self.get_logger().info(f'Connected to ESP32 on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {port}: {e}')
            self.connected = False
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to left/right wheel speeds and send to ESP32."""
        if not self.connected:
            return
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Differential drive kinematics
        # v_left = linear_vel - (angular_vel * wheel_base / 2)
        # v_right = linear_vel + (angular_vel * wheel_base / 2)
        v_left = linear_vel - (angular_vel * self.wheel_base / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        # Convert to motor speeds (you may need to scale these)
        # For now, sending as-is. Adjust scaling factor as needed.
        speed_left = int(v_left * 100)  # Scale factor
        speed_right = int(v_right * 100)
        
        # Send as 'L,R\n' format
        command = f'{speed_left},{speed_right}\n'
        
        try:
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent to ESP32: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.connected = False
            self.attempt_reconnect()
    
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
            
            time.sleep(0.01)  # Small delay to prevent CPU spin
    
    def parse_serial_data(self, line):
        """Parse incoming serial data: 'enc1,enc2,ax,ay,az,gx,gy,gz'"""
        try:
            parts = line.split(',')
            if len(parts) != 8:
                self.get_logger().warning(f'Invalid data format: {line}')
                return
            
            enc1 = int(parts[0])
            enc2 = int(parts[1])
            ax = float(parts[2])
            ay = float(parts[3])
            az = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])
            
            # Publish odometry and IMU
            self.publish_odometry(enc1, enc2)
            self.publish_imu(ax, ay, az, gx, gy, gz)
            
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Error parsing data "{line}": {e}')
    
    def publish_odometry(self, enc1, enc2):
        """Compute and publish odometry from encoder values."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate change in encoder counts
        d_enc1 = enc1 - self.last_enc1
        d_enc2 = enc2 - self.last_enc2
        
        # Convert encoder ticks to distance (adjust counts_per_rev as needed)
        counts_per_rev = 360.0  # Example: adjust to your encoder
        d_left = (d_enc1 / counts_per_rev) * 2.0 * 3.14159 * self.wheel_radius
        d_right = (d_enc2 / counts_per_rev) * 2.0 * 3.14159 * self.wheel_radius
        
        # Calculate linear and angular displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base
        
        # Update pose
        self.x += d_center * float(rclpy.math.cos(self.theta))
        self.y += d_center * float(rclpy.math.sin(self.theta))
        self.theta += d_theta
        
        # Create Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(rclpy.math.sin(self.theta / 2.0))
        odom.pose.pose.orientation.w = float(rclpy.math.cos(self.theta / 2.0))
        
        # Velocity
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt
        
        self.odom_pub.publish(odom)
        
        # Update state
        self.last_enc1 = enc1
        self.last_enc2 = enc2
        self.last_time = current_time
    
    def publish_imu(self, ax, ay, az, gx, gy, gz):
        """Publish IMU data."""
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        
        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        
        # Angular velocity (rad/s)
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        
        # Covariance (set to -1 if unknown)
        imu.linear_acceleration_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.orientation_covariance[0] = -1.0
        
        self.imu_pub.publish(imu)
    
    def attempt_reconnect(self):
        """Attempt to reconnect to serial port."""
        self.get_logger().warn('Attempting to reconnect...')
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
        
        time.sleep(2.0)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.connect_serial(port, baud, timeout)
    
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
