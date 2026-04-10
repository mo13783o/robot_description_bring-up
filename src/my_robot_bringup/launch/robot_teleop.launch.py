from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.61'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed_step',
        default_value='0.1'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed_step',
        default_value='0.5'
    )

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
            ('/odom','/odom/raw'),
            ('/imu','/imu/data'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    keyboard_teleop_node = Node(
        package='my_robot_bringup',
        executable='keyboard_teleop_node.py',
        name='keyboard_teleop_node',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'linear_speed_step': LaunchConfiguration('linear_speed_step'),
            'angular_speed_step': LaunchConfiguration('angular_speed_step'),
            'max_linear_speed': 0.5,
            'max_angular_speed': 2.0,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        linear_speed_arg,
        angular_speed_arg,
        esp32_bridge_node,
        keyboard_teleop_node,
    ])
