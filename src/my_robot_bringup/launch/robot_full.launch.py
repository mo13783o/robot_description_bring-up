"""
Master Launch File for Complete Robot System
==============================================
Launches all necessary nodes for robot operation:
- Robot description (URDF, Robot State Publisher)
- ESP32 serial bridge (hardware interface)
- Robot localization (EKF for sensor fusion)
- Keyboard teleop (optional, for manual control)

Usage:
    # Full system with teleop
    ros2 launch my_robot_bringup robot_full.launch.py
    
    # Without teleop (for autonomous operation)
    ros2 launch my_robot_bringup robot_full.launch.py run_teleop:=false
    
    # Custom serial port
    ros2 launch my_robot_bringup robot_full.launch.py serial_port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the complete robot launch description."""
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    # Serial port configuration
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 connection (e.g., /dev/ttyUSB0, /dev/ttyACM0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate for ESP32 communication'
    )
    
    # Robot physical parameters
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.61',
        description='Distance between left and right wheels in meters'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Radius of the wheels in meters'
    )
    
    # Teleop control
    run_teleop_arg = DeclareLaunchArgument(
        'run_teleop',
        default_value='true',
        description='Launch keyboard teleop node (true/false)'
    )
    
    # Teleop speed settings
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed_step',
        default_value='0.1',
        description='Default linear velocity for teleop in m/s'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed_step',
        default_value='0.5',
        description='Default angular velocity for teleop in rad/s'
    )
    
    # Localization settings
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Enable EKF localization (true/false)'
    )
    
    # ========================================================================
    # PACKAGE PATHS
    # ========================================================================
    
    # Get package directories
    my_robot_description_share = FindPackageShare('my_robot_description')
    my_robot_bringup_share = FindPackageShare('my_robot_bringup')
    
    # EKF configuration file
    ekf_config_file = PathJoinSubstitution([
        my_robot_bringup_share,
        'config',
        'ekf.yaml'
    ])
    
    # ========================================================================
    # NODE 1: ROBOT DESCRIPTION (URDF, ROBOT STATE PUBLISHER)
    # ========================================================================
    
    # Include the view_robot.launch.py from my_robot_description package
    # This handles URDF loading and Robot State Publisher
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                my_robot_description_share,
                'launch',
                'view_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',  # Using real hardware, not simulation
        }.items()
    )
    
    # ========================================================================
    # NODE 2: ESP32 BRIDGE (HARDWARE INTERFACE)
    # ========================================================================
    
    # ESP32 serial bridge node for motor control and sensor data
    esp32_bridge_node = Node(
        package='my_robot_bringup',
        executable='esp32_bridge_node.py',
        name='esp32_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
        }],
        remappings=[
            # ESP32 bridge publishes to /odom and /imu
            # These are consumed by the EKF node
            ('/cmd_vel', '/cmd_vel'),  # Subscribes from teleop
            ('/odom', '/odom/raw'),     # Publish raw odometry (before EKF)
            ('/imu', '/imu/data'),      # Publish IMU data
        ],
        respawn=True,  # Auto-restart if node crashes
        respawn_delay=2.0,
    )
    
    # ========================================================================
    # NODE 3: ROBOT LOCALIZATION (EKF - SENSOR FUSION)
    # ========================================================================
    
    # Extended Kalman Filter for fusing odometry and IMU
    # Provides /odom -> /base_link transform
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            # EKF subscribes to raw odometry and IMU
            ('/odometry/filtered', '/odom'),  # Publish filtered odometry
            ('/odom0', '/odom/raw'),          # Subscribe to raw odom from ESP32
            ('/imu0', '/imu/data'),           # Subscribe to IMU from ESP32
        ],
        condition=IfCondition(LaunchConfiguration('use_ekf'))
    )
    
    # ========================================================================
    # NODE 4: KEYBOARD TELEOP (OPTIONAL - FOR MANUAL CONTROL)
    # ========================================================================
    
    # Keyboard teleoperation node (opens in separate terminal if run_teleop=true)
    keyboard_teleop_node = Node(
        package='my_robot_bringup',
        
        executable='keyboard_teleop_node.py',

        name='keyboard_teleop_node',
        output='screen',
        prefix='xterm -e',  # Open in new terminal window (requires xterm)
        parameters=[{
            'linear_speed_step': LaunchConfiguration('linear_speed_step'),
            'angular_speed_step': LaunchConfiguration('angular_speed_step'),
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
        }],
        remappings=[
            # Teleop publishes velocity commands
            ('/cmd_vel', '/cmd_vel'),
        ],
        condition=IfCondition(LaunchConfiguration('run_teleop'))
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baud_rate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        run_teleop_arg,
        linear_speed_arg,
        angular_speed_arg,
        use_ekf_arg,
        
        # Launch nodes in order
        robot_description_launch,  # 1. Load URDF and start Robot State Publisher
        esp32_bridge_node,         # 2. Start hardware interface
        ekf_localization_node,     # 3. Start sensor fusion
        keyboard_teleop_node,      # 4. Start teleop (if enabled)
    ])
