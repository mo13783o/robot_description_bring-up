# Complete Robot System Setup Guide

## Package Structure

After integration, your `my_robot_bringup` package should look like this:

```
my_robot_bringup/
├── CMakeLists.txt                    ← BUILD CONFIGURATION
├── package.xml                       ← PACKAGE DEPENDENCIES
├── my_robot_bringup/
│   ├── __init__.py
│   ├── esp32_bridge_node.py         ← Hardware interface node
│   └── keyboard_teleop_node.py      ← Teleop control node
├── launch/
│   ├── robot_full.launch.py         ← MASTER LAUNCH FILE (NEW)
│   ├── robot_teleop.launch.py       ← Teleop + ESP32 only
│   └── robot_bridge.launch.py       ← ESP32 bridge only
├── config/
│   └── ekf.yaml                      ← EKF configuration (NEW)
├── rviz/
│   └── robot_view.rviz              ← RViz config (optional)
└── README.md
```

## Installation Steps

### Step 1: Install Dependencies

```bash
# ROS 2 packages
sudo apt update
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    xterm

# Python packages
pip3 install pyserial --break-system-packages
```

### Step 2: Copy Files to Package

```bash
cd ~/ros2_ws/src/my_robot_bringup

# Create directories if they don't exist
mkdir -p my_robot_bringup launch config

# Copy Python nodes
cp /path/to/esp32_bridge_node.py my_robot_bringup/
cp /path/to/keyboard_teleop_node.py my_robot_bringup/
chmod +x my_robot_bringup/*.py

# Copy launch files
cp /path/to/robot_full.launch.py launch/
cp /path/to/robot_teleop.launch.py launch/
cp /path/to/robot_bridge.launch.py launch/

# Copy config files
cp /path/to/ekf.yaml config/

# Copy build files
cp /path/to/CMakeLists.txt .
cp /path/to/package.xml .
```

### Step 3: Update setup.py

Edit `~/ros2_ws/src/my_robot_bringup/setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot bringup package with ESP32 integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = my_robot_bringup.esp32_bridge_node:main',
            'keyboard_teleop = my_robot_bringup.keyboard_teleop_node:main',
        ],
    },
)
```

### Step 4: Build the Package

```bash
cd ~/ros2_ws

# Build only this package
colcon build --packages-select my_robot_bringup

# Or build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Verify Installation

```bash
# Check if launch files are installed
ros2 launch my_robot_bringup <TAB><TAB>
# Should show: robot_full.launch.py, robot_teleop.launch.py, robot_bridge.launch.py

# Check if nodes are available
ros2 run my_robot_bringup <TAB><TAB>
# Should show: esp32_bridge, keyboard_teleop

# Check if config is accessible
ros2 pkg prefix my_robot_bringup
# Then check: <prefix>/share/my_robot_bringup/config/ekf.yaml
```

## Running the Complete System

### Method 1: Master Launch File (Recommended)

```bash
# Full system with teleop (default)
ros2 launch my_robot_bringup robot_full.launch.py

# Without teleop (for autonomous mode)
ros2 launch my_robot_bringup robot_full.launch.py run_teleop:=false

# Custom serial port
ros2 launch my_robot_bringup robot_full.launch.py serial_port:=/dev/ttyACM0

# Custom robot parameters
ros2 launch my_robot_bringup robot_full.launch.py \
    serial_port:=/dev/ttyUSB0 \
    wheel_base:=0.25 \
    wheel_radius:=0.06 \
    linear_speed_step:=0.15
```

### Method 2: Manual Launch (for debugging)

```bash
# Terminal 1: Robot Description
ros2 launch my_robot_description view_robot.launch.py

# Terminal 2: ESP32 Bridge
ros2 run my_robot_bringup esp32_bridge

# Terminal 3: EKF Localization
ros2 launch robot_localization ekf.launch.py \
    params_file:=$(ros2 pkg prefix my_robot_bringup)/share/my_robot_bringup/config/ekf.yaml

# Terminal 4: Keyboard Teleop
ros2 run my_robot_bringup keyboard_teleop
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROBOT SYSTEM                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐         /cmd_vel          ┌────────────────┐  │
│  │  Keyboard    │─────────────────────────→ │  ESP32 Bridge  │  │
│  │   Teleop     │                            │     Node       │  │
│  └──────────────┘                            └────────────────┘  │
│                                                    │              │
│                                                    │              │
│                                    /odom/raw ──────┤              │
│                                    /imu/data ──────┤              │
│                                                    ↓              │
│                                              ┌────────────────┐  │
│                                              │  EKF Filter    │  │
│                                              │     Node       │  │
│                                              └────────────────┘  │
│                                                    │              │
│                                                    │ /odom        │
│                                                    │              │
│                                                    ↓              │
│  ┌──────────────┐                            ┌────────────────┐  │
│  │    Robot     │←─── joint_states ──────── │  Robot State   │  │
│  │  Description │                            │   Publisher    │  │
│  │    (URDF)    │                            └────────────────┘  │
│  └──────────────┘                                                │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘

TF TREE:
    map (future SLAM)
     └─ odom (from EKF)
         └─ base_link
             ├─ left_wheel
             ├─ right_wheel
             ├─ imu_link
             └─ lidar_link (if applicable)
```

## Topic Flow

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | keyboard_teleop | esp32_bridge | Velocity commands |
| `/odom/raw` | nav_msgs/Odometry | esp32_bridge | ekf_node | Raw wheel odometry |
| `/imu/data` | sensor_msgs/Imu | esp32_bridge | ekf_node | IMU sensor data |
| `/odom` | nav_msgs/Odometry | ekf_node | (navigation) | Filtered odometry |
| `/joint_states` | sensor_msgs/JointState | robot_state_pub | - | Joint positions |
| `/tf` | tf2_msgs/TFMessage | ekf_node + robot_state_pub | All | Transform tree |

## Transform Tree Responsibilities

### Robot State Publisher
- Publishes: `base_link` → `left_wheel`, `right_wheel`, `imu_link`, etc.
- Source: URDF file from `my_robot_description`

### EKF Node
- Publishes: `odom` → `base_link`
- Source: Fused odometry and IMU data
- **CRITICAL:** Set `publish_tf: true` in `ekf.yaml`

### Future SLAM Node (e.g., slam_toolbox)
- Will publish: `map` → `odom`
- Not included in this setup yet

## Troubleshooting

### Issue: "No transform from odom to base_link"

**Check:**
```bash
# Verify EKF is publishing the transform
ros2 run tf2_ros tf2_echo odom base_link

# Check if EKF node is running
ros2 node list | grep ekf

# Verify ekf.yaml has publish_tf: true
```

**Solution:**
- Ensure `publish_tf: true` in `config/ekf.yaml`
- Make sure EKF node is running and receiving data

### Issue: "EKF not receiving odometry or IMU"

**Check:**
```bash
# Check topic remapping
ros2 topic echo /odom/raw
ros2 topic echo /imu/data

# Verify ESP32 is publishing
ros2 topic hz /odom/raw
ros2 topic hz /imu/data
```

**Solution:**
- Verify ESP32 serial connection
- Check topic names in launch file remapping
- Ensure ESP32 code is sending data in correct format

### Issue: "Keyboard teleop not working"

**Check:**
```bash
# Verify node is running
ros2 node list | grep teleop

# Check if publishing
ros2 topic echo /cmd_vel

# Terminal issues?
echo $TERM
```

**Solution:**
- Make sure you're in the terminal running teleop (not a different terminal)
- If using launch file, ensure `xterm` is installed
- Try running manually: `ros2 run my_robot_bringup keyboard_teleop`

### Issue: "view_robot.launch.py not found"

**Error:** `Package 'my_robot_description' not found`

**Solution:**
```bash
# Build the description package first
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash

# Then build bringup package
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

### Issue: "Serial port permission denied"

**Error:** `PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'`

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or temporarily fix
sudo chmod 666 /dev/ttyUSB0

# Log out and back in for group change to take effect
```

## Visualization with RViz

```bash
# Run the full system
ros2 launch my_robot_bringup robot_full.launch.py

# In another terminal, start RViz
rviz2

# Add displays:
# - RobotModel (uses /robot_description)
# - TF (shows transform tree)
# - Odometry (topic: /odom)
# - Path (topic: /odom to see robot trajectory)
```

### Recommended RViz Configuration

1. **Fixed Frame:** `odom`
2. **Displays to add:**
   - RobotModel
   - TF (show all frames)
   - Odometry (topic: `/odom`, color: green)
   - Odometry (topic: `/odom/raw`, color: red) - for comparison
   - Axes (at `odom`, `base_link`)

## Performance Tuning

### EKF Tuning

Edit `config/ekf.yaml` to adjust sensor trust:

```yaml
# Trust odometry more (reduce noise values)
odom0_twist_rejection_threshold: 0.5  # Lower = more trust

# Trust IMU more
imu0_twist_rejection_threshold: 0.3   # Lower = more trust

# Increase update frequency
frequency: 50.0  # Hz (was 30.0)
```

### Serial Communication

If experiencing lag or dropped packets:

```python
# In esp32_bridge_node.py, adjust timeout
self.declare_parameter('timeout', 0.5)  # Increase from 1.0

# In ESP32 Arduino code, adjust publish rate
const unsigned long PUBLISH_INTERVAL = 20; // 50 Hz instead of 20 Hz
```

## Next Steps

1. **Add SLAM:** Integrate `slam_toolbox` for mapping
2. **Add Navigation:** Integrate `nav2` for autonomous navigation
3. **Add Sensors:** Integrate lidar, camera, etc.
4. **Add Simulation:** Create Gazebo simulation for testing
5. **Add Autonomous Behaviors:** Implement path planning, obstacle avoidance

## Quick Reference Commands

```bash
# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch full system
ros2 launch my_robot_bringup robot_full.launch.py

# View TF tree
ros2 run tf2_tools view_frames

# Monitor topics
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /cmd_vel

# Check node graph
rqt_graph

# Record data
ros2 bag record -a

# Replay data
ros2 bag play <bag_file>
```

## Support

For issues or questions:
1. Check this README
2. Verify all dependencies are installed
3. Check ROS 2 logs: `ros2 log`
4. Enable debug output in nodes
5. Use RViz to visualize data flow
