# ESP32 ROS 2 Serial Bridge Setup

## Installation

### 1. Install Dependencies
```bash
pip3 install pyserial
```

### 2. ROS 2 Package Setup
Copy `esp32_bridge_node.py` to your ROS 2 workspace:
```bash
# Assuming you're in your workspace
cd ~/ros2_ws/src/your_package/your_package/
cp esp32_bridge_node.py .
chmod +x esp32_bridge_node.py
```

Or create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python esp32_bridge
cd esp32_bridge/esp32_bridge
cp esp32_bridge_node.py .
chmod +x esp32_bridge_node.py
```

### 3. Update `setup.py`
Add to your package's `setup.py`:
```python
entry_points={
    'console_scripts': [
        'esp32_bridge = esp32_bridge.esp32_bridge_node:main',
    ],
},
```

### 4. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select esp32_bridge
source install/setup.bash
```

## ESP32 Setup

### 1. Arduino IDE Configuration
- Install ESP32 board support: https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
- Install Wire library (usually pre-installed)

### 2. Upload Code
1. Open `esp32_robot_controller.ino` in Arduino IDE
2. Select your ESP32 board (e.g., "ESP32 Dev Module")
3. Select the correct COM port
4. Upload the sketch

### 3. Hardware Connections
**IMU (MPU6050):**
- VCC → 3.3V
- GND → GND
- SDA → GPIO 21
- SCL → GPIO 22

**Motors (L298N example):**
- Left PWM → GPIO 25
- Left DIR → GPIO 26
- Right PWM → GPIO 27
- Right DIR → GPIO 14

**Encoders:**
- Left A → GPIO 32
- Left B → GPIO 33
- Right A → GPIO 34
- Right B → GPIO 35

## Running the Node

### Basic Usage
```bash
ros2 run esp32_bridge esp32_bridge
```

### With Custom Parameters
```bash
ros2 run esp32_bridge esp32_bridge \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p wheel_base:=0.2 \
  -p wheel_radius:=0.05
```

### Launch File Example
Create `launch/esp32_bridge.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp32_bridge',
            executable='esp32_bridge',
            name='esp32_bridge_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'wheel_base': 0.2,
                'wheel_radius': 0.05,
            }],
            output='screen'
        )
    ])
```

## Testing

### 1. Check Serial Connection
```bash
# Find your ESP32 port
ls /dev/ttyUSB* /dev/ttyACM*

# Give permissions (if needed)
sudo chmod 666 /dev/ttyUSB0
```

### 2. Monitor Topics
```bash
# Terminal 1: Run the node
ros2 run esp32_bridge esp32_bridge

# Terminal 2: Check odometry
ros2 topic echo /odom

# Terminal 3: Check IMU
ros2 topic echo /imu

# Terminal 4: Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

### 3. Visualize in RViz
```bash
rviz2
# Add displays:
# - Odometry: /odom
# - TF
# - Axes for imu_link
```

## Calibration

### Encoder Counts Per Revolution
Adjust in the Python node:
```python
counts_per_rev = 360.0  # Change this to match your encoders
```

### Motor Speed Scaling
Adjust the scaling factor in `cmd_vel_callback()`:
```python
speed_left = int(v_left * 100)  # Modify multiplier
```

### IMU Calibration
Place robot stationary and record offsets, then update in Arduino code:
```cpp
float accel_offset_x = 0.05;  // Your values
float gyro_offset_x = 0.02;
```

## Troubleshooting

### Serial Port Permission Denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Node Can't Find Serial Port
```bash
# Check connected devices
dmesg | grep tty
ls -l /dev/ttyUSB*
```

### No Data Published
- Check Arduino Serial Monitor (115200 baud)
- Verify data format matches: `enc1,enc2,ax,ay,az,gx,gy,gz`
- Check wire connections to sensors

### Motors Not Responding
- Verify motor driver power supply
- Check GPIO pin numbers match your wiring
- Test motors with simple Arduino sketch first

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port device |
| `baud_rate` | `115200` | Serial communication speed |
| `timeout` | `1.0` | Serial read timeout (seconds) |
| `wheel_base` | `0.2` | Distance between wheels (meters) |
| `wheel_radius` | `0.05` | Wheel radius (meters) |

## Topics

**Subscribed:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Published:**
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/imu` (sensor_msgs/Imu) - IMU sensor data

## Communication Protocol

**ROS 2 → ESP32:**
```
<speed_left>,<speed_right>\n
```
Example: `50,-30\n` (left forward, right backward)

**ESP32 → ROS 2:**
```
<enc1>,<enc2>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
```
Example: `1234,5678,0.05,0.02,9.81,0.001,0.002,0.003\n`
