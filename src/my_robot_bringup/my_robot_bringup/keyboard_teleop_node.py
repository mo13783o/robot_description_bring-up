#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class KeyboardTeleopNode(Node):
    """
    Keyboard teleoperation node for differential drive robots.
    Compatible with esp32_bridge_node.py via /cmd_vel topic.
    """
    
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Parameters - matching ESP32 bridge robot configuration
        self.declare_parameter('linear_speed_step', 0.1)  # m/s
        self.declare_parameter('angular_speed_step', 0.5)  # rad/s
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('speed_increment', 0.05)  # m/s per key press
        self.declare_parameter('angular_increment', 0.25)  # rad/s per key press
        
        self.linear_speed = self.get_parameter('linear_speed_step').value
        self.angular_speed = self.get_parameter('angular_speed_step').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.angular_increment = self.get_parameter('angular_increment').value
        
        # Publisher to /cmd_vel (same topic ESP32 bridge subscribes to)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocity state
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Key bindings
        self.move_bindings = {
            'w': (1, 0),   # Forward
            'W': (1, 0),
            's': (-1, 0),  # Backward
            'S': (-1, 0),
            'a': (0, 1),   # Turn left
            'A': (0, 1),
            'd': (0, -1),  # Turn right
            'D': (0, -1),
            'q': (1, 1),   # Forward + left
            'Q': (1, 1),
            'e': (1, -1),  # Forward + right
            'E': (1, -1),
            'z': (-1, 1),  # Backward + left
            'Z': (-1, 1),
            'c': (-1, -1), # Backward + right
            'C': (-1, -1),
        }
        
        # Arrow keys (ANSI escape sequences)
        self.arrow_keys = {
            '\x1b[A': (1, 0),   # Up arrow
            '\x1b[B': (-1, 0),  # Down arrow
            '\x1b[D': (0, 1),   # Left arrow
            '\x1b[C': (0, -1),  # Right arrow
        }
        
        self.speed_bindings = {
            '+': (1.1, 1.1),   # Increase speed
            '=': (1.1, 1.1),
            '-': (0.9, 0.9),   # Decrease speed
            '_': (0.9, 0.9),
        }
        
        self.get_logger().info('Keyboard Teleop Node initialized')
        self.print_menu()
    
    def get_key(self, timeout=0.1):
        """Read a single keypress from stdin with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Check for escape sequences (arrow keys)
            if key == '\x1b':
                # Read the next two characters for arrow keys
                key += sys.stdin.read(2)
            return key
        return ''
    
    def print_menu(self):
        """Print control instructions to terminal."""
        msg = """
╔════════════════════════════════════════════════════════════════╗
║              KEYBOARD TELEOP - DIFFERENTIAL DRIVE              ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  MOVEMENT CONTROLS (WASD or Arrow Keys):                      ║
║  ┌─────────┬─────────┬─────────┐                              ║
║  │    Q    │    W    │    E    │     Q: Forward + Left        ║
║  │  ↖      │    ↑    │    ↗  │     W: Forward               ║
║  ├─────────┼─────────┼─────────┤     E: Forward + Right       ║
║  │    A    │         │    D    │     A: Rotate Left           ║
║  │  ←      │         │    →  │     D: Rotate Right          ║
║  ├─────────┼─────────┼─────────┤     S: Backward              ║
║  │    Z    │    S    │    C    │     Z: Backward + Left       ║
║  │  ↙      │    ↓    │    ↘  │     C: Backward + Right      ║
║  └─────────┴─────────┴─────────┘                              ║
║                                                                ║
║  Arrow Keys: ↑ ↓ ← → (same as W S A D)                        ║
║                                                                ║
╠════════════════════════════════════════════════════════════════╣
║  SPEED CONTROLS:                                               ║
║    +/=  : Increase speed by 10%%                               ║
║    -/_  : Decrease speed by 10%%                               ║
║                                                                ║
║  EMERGENCY:                                                    ║
║    SPACE: STOP (all motors halt immediately)                   ║
║    X/x  : STOP (alternative key)                               ║
║                                                                ║
║  EXIT:                                                         ║
║    Ctrl+C : Quit teleop node                                   ║
║                                                                ║
╠════════════════════════════════════════════════════════════════╣
║  CURRENT SETTINGS:                                             ║
║    Linear Speed  : %.2f m/s                                    ║
║    Angular Speed : %.2f rad/s                                  ║
║    Max Linear    : %.2f m/s                                    ║
║    Max Angular   : %.2f rad/s                                  ║
║                                                                ║
║  Robot Configuration:                                          ║
║    Wheel Base    : 0.2 m                                       ║
║    Wheel Radius  : 0.05 m                                      ║
║                                                                ║
╠════════════════════════════════════════════════════════════════╣
║  Publishing to: /cmd_vel (geometry_msgs/Twist)                 ║
║  Compatible with: esp32_bridge_node.py                         ║
╚════════════════════════════════════════════════════════════════╝

Ready! Press any movement key to start...
""" % (self.linear_speed, self.angular_speed, self.max_linear, self.max_angular)
        print(msg)
    
    def update_speed_display(self):
        """Update speed display in terminal."""
        print(f"\r\033[KLinear: {self.linear_speed:.2f} m/s | Angular: {self.angular_speed:.2f} rad/s | "
              f"Target: L={self.target_linear:.2f} A={self.target_angular:.2f}  ", end='', flush=True)
    
    def publish_twist(self, linear, angular):
        """Publish Twist message to /cmd_vel."""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular)
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f'Published: linear={linear:.2f}, angular={angular:.2f}')
    
    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    break
                
                if key in self.move_bindings:
                    # Standard WASD/QEZC keys
                    linear_dir, angular_dir = self.move_bindings[key]
                    self.target_linear = linear_dir * self.linear_speed
                    self.target_angular = angular_dir * self.angular_speed
                    self.publish_twist(self.target_linear, self.target_angular)
                    self.update_speed_display()
                
                elif key in self.arrow_keys:
                    # Arrow keys
                    linear_dir, angular_dir = self.arrow_keys[key]
                    self.target_linear = linear_dir * self.linear_speed
                    self.target_angular = angular_dir * self.angular_speed
                    self.publish_twist(self.target_linear, self.target_angular)
                    self.update_speed_display()
                
                elif key in self.speed_bindings:
                    # Adjust speed
                    linear_mult, angular_mult = self.speed_bindings[key]
                    self.linear_speed *= linear_mult
                    self.angular_speed *= angular_mult
                    
                    # Clamp to max values
                    self.linear_speed = min(self.linear_speed, self.max_linear)
                    self.angular_speed = min(self.angular_speed, self.max_angular)
                    
                    # Ensure minimum speed
                    self.linear_speed = max(self.linear_speed, 0.01)
                    self.angular_speed = max(self.angular_speed, 0.1)
                    
                    print(f"\n\033[K>>> Speed updated: Linear={self.linear_speed:.2f} m/s, "
                          f"Angular={self.angular_speed:.2f} rad/s")
                    self.update_speed_display()
                
                elif key in [' ', 'x', 'X']:
                    # Emergency stop
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.publish_twist(0.0, 0.0)
                    print("\n\033[K>>> EMERGENCY STOP! All motors halted.")
                    self.update_speed_display()
                
                elif key == '':
                    # Timeout - no key pressed, send zero velocity (dead man's switch)
                    if self.target_linear != 0.0 or self.target_angular != 0.0:
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.publish_twist(0.0, 0.0)
                
                else:
                    # Invalid key
                    if key and key not in ['\r', '\n']:
                        print(f"\n\033[K>>> Unknown key: '{key}' - Press W/A/S/D or arrows to move")
                        self.update_speed_display()
        
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
        
        finally:
            # Restore terminal settings and send stop command
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.publish_twist(0.0, 0.0)
            print("\n\nTeleop node stopped. Robot halted.")
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        # Send stop command
        self.publish_twist(0.0, 0.0)
        
        # Restore terminal
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        except:
            pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
