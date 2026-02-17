import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Identify the path to your config file
    # This matches your screenshot path: my_robot_bringup/config/ekf.yaml
    ekf_config_path = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'ekf.yaml'
    )

    # 2. Define the EKF Node
    # This node implements the Jacobian logic discussed in your analysis
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # 3. Return the Launch Description
    return LaunchDescription([
        ekf_node
    ])