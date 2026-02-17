import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_bringup'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths to configuration files
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # Standard SLAM Toolbox configuration
    slam_config_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Process the URDF (Robot Model)
    xacro_file = os.path.join(pkg_share, 'urdf', 'final2.xml')
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    # 1. Robot State Publisher (Publishes the 3D structure)
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # 2. Combined Driver Node (Handles ESP32 Serial -> Odom & IMU)
    node_combined_driver = Node(
        package=pkg_name,
        executable='encoder_odom_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
            'publish_tf': False  # CRITICAL: Let the EKF handle the odom->base_link transform
        }]
    )

    # 3. EKF Node (Fuses Encoder and IMU data)
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # 4. RPLIDAR Driver (LIDAR is usually on USB0)
    node_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'scan_mode': 'Standard'
        }]
    )

    # 5. SLAM Toolbox (Asynchronous Mapping)
    node_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_config_path,
            {
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan'
            }
        ]
    )

    # 6. RViz2 (Visualizer)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_rsp,
        node_combined_driver,
        node_ekf,
        # Delaying sensors and SLAM to ensure the TF tree is ready
        TimerAction(period=2.0, actions=[node_rplidar]),
        TimerAction(period=5.0, actions=[node_slam]),
        node_rviz
    ])