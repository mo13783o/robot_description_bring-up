import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Paths to Packages and Files
    bringup_dir = get_package_share_directory('my_robot_bringup')
    description_dir = get_package_share_directory('my_robot_description')
    
    ekf_config_path = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    # 2. ESP32 Bridge Node (Your Driver)
    # This node handles /cmd_vel, /odom (raw), and /imu
    esp32_bridge = Node(
        package='my_robot_bringup', # Ensure this matches your package name
        executable='esp32_bridge_node.py',
        name='esp32_bridge_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'wheel_base': 0.2,   # Update based on your physical robot
            'wheel_radius': 0.05 # Update based on your physical robot
        }],
        output='screen'
    )

    # 3. Robot State Publisher (from your description package)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            description_dir, 'launch', 'view_robot.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # 4. EKF Node (The "Brain" for Localization)
    # Uses the mathematically optimized Jacobian logic from your ekf.yaml
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # 5. Mapping (SLAM Toolbox)
    # Usually better to delay SLAM until the EKF has stabilized
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'mapping.launch.py'
        )])
    )
    
    delayed_mapping = TimerAction(
        period=3.0,
        actions=[mapping_launch]
    )

    return LaunchDescription([
        esp32_bridge,
        robot_state_publisher,
        ekf_filter_node,
        delayed_mapping
    ])