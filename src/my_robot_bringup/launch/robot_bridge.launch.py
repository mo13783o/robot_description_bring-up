"""
Alternative launch file - runs teleop in same terminal (no xterm needed).
Use this if you don't have xterm installed.

Usage:
    # Terminal 1: Launch ESP32 bridge
    ros2 launch my_robot_bringup robot_bridge.launch.py
    
    # Terminal 2: Run teleop separately
    ros2 run my_robot_bringup keyboard_teleop
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch only the ESP32 bridge node."""
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.2',
        description='Distance between wheels in meters'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='Wheel radius in meters'
    )
    
    esp32_bridge_node = Node(
        package='my_robot_bringup',  # Change to your package name
        executable='esp32_bridge_node.py',
        name='esp32_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        esp32_bridge_node,
    ])
