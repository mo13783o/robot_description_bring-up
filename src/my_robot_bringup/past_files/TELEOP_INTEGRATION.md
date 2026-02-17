# Keyboard Teleop Node - Integration Guide

## Overview
This keyboard teleop node provides manual control for your differential drive robot through the `/cmd_vel` topic, which is consumed by your `esp32_bridge_node.py`.

## Package Structure

### Where to Place the File

**Option 1: Add to existing package** (e.g., `my_robot_bringup`)
```
my_robot_bringup/
├── my_robot_bringup/
│   ├── __init__.py
│   ├── esp32_bridge_node.py      ← Your existing ESP32 node
│   └── keyboard_teleop_node.py   ← NEW: Place here
├── setup.py                       ← UPDATE: Add entry point
├── package.xml
└── README.md
```

**Option 2: Create dedicated teleop package**
```
my_robot_teleop/
├── my_robot_teleop/
│   ├── __init__.py
│   └── keyboard_teleop_node.py   ← Place here
├── setup.py
├── package.xml
└── README.md
```

## Installation Steps

### Step 1: Copy the File
```bash
cd ~/ros2_ws/src/my_robot_bringup/my_robot_bringup/
cp /path/to/keyboard_teleop_node.py .
chmod +x keyboard_teleop_node.py
```

### Step 2: Update `setup.py`
Edit `~/ros2_ws/src/my_robot_bringup/setup.py` and add the entry point:

```python
from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot bringup package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = my_robot_bringup.esp32_bridge_node:main',
            'keyboard_teleop = my_robot_bringup.keyboard_teleop_node:main',  # ← ADD THIS
        ],
    },
)
```

### Step 3: Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

## Running the Teleop Node

### Terminal 1: Start ESP32 Bridge
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_bringup esp32_bridge
```

### Terminal 2: Start Keyboard Teleop
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_robot_bringup keyboard_teleop
```

### With Custom Parameters
```bash
ros2 run my_robot_bringup keyboard_teleop \
  --ros-args \
  -p linear_speed_step:=0.2 \
  -p angular_speed_step:=0.8 \
  -p max_linear_speed:=1.0 \
  -p max_angular_speed:=3.0
```

## Launch File (Recommended)

Create `launch/robot_teleop.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ESP32 Bridge Node
        Node(
            package='my_robot_bringup',
            executable='esp32_bridge',
            name='esp32_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'wheel_base': 0.2,
                'wheel_radius': 0.05,
            }]
        ),
        
        # Keyboard Teleop Node
        Node(
            package='my_robot_bringup',
            executable='keyboard_teleop',
            name='keyboard_teleop_node',
            output='screen',
            prefix='xterm -e',  # Opens in separate terminal window
            parameters=[{
                'linear_speed_step': 0.1,
                'angular_speed_step': 0.5,
                'max_linear_speed': 0.5,
                'max_angular_speed': 2.0,
            }]
        ),
    ])
```

**Launch everything together:**
```bash
ros2 launch my_robot_bringup robot_teleop.launch.py
```

## Control Scheme

### Movement Keys
| Key | Action | Linear (m/s) | Angular (rad/s) |
|-----|--------|--------------|-----------------|
| W / ↑ | Forward | +speed | 0 |
| S / ↓ | Backward | -speed | 0 |
| A / ← | Turn Left | 0 | +speed |
| D / → | Turn Right | 0 | -speed |
| Q | Forward + Left | +speed | +speed |
| E | Forward + Right | +speed | -speed |
| Z | Backward + Left | -speed | +speed |
| C | Backward + Right | -speed | -speed |

### Speed Control
| Key | Action |
|-----|--------|
| + or = | Increase speed by 10% |
| - or _ | Decrease speed by 10% |

### Emergency
| Key | Action |
|-----|--------|
| SPACE | Emergency stop (0, 0) |
| X or x | Emergency stop (0, 0) |

### Exit
| Key | Action |
|-----|--------|
| Ctrl+C | Quit teleop and stop robot |

## Velocity Calculations

The node publishes `geometry_msgs/Twist` messages with:
- **Linear velocity** (`linear.x`): Forward/backward speed in m/s
- **Angular velocity** (`angular.z`): Rotation speed in rad/s

These values are directly consumed by your `esp32_bridge_node.py`, which converts them to differential wheel speeds using:

```python
v_left = linear_vel - (angular_vel * wheel_base / 2.0)
v_right = linear_vel + (angular_vel * wheel_base / 2.0)
```

### Example Calculations
Given your robot parameters:
- `wheel_base = 0.2 m`
- `wheel_radius = 0.05 m`

**Forward motion** (W key with default 0.1 m/s):
```
linear = 0.1, angular = 0.0
v_left = 0.1 - (0.0 * 0.2 / 2) = 0.1 m/s
v_right = 0.1 + (0.0 * 0.2 / 2) = 0.1 m/s
→ Both wheels same speed, robot moves forward
```

**Turn in place** (A key with default 0.5 rad/s):
```
linear = 0.0, angular = 0.5
v_left = 0.0 - (0.5 * 0.2 / 2) = -0.05 m/s
v_right = 0.0 + (0.5 * 0.2 / 2) = 0.05 m/s
→ Wheels turn opposite directions, robot rotates left
```

**Arc turn** (Q key: forward + left):
```
linear = 0.1, angular = 0.5
v_left = 0.1 - (0.5 * 0.2 / 2) = 0.05 m/s
v_right = 0.1 + (0.5 * 0.2 / 2) = 0.15 m/s
→ Right wheel faster, robot curves left while moving forward
```

## Testing Communication

### Verify Topic Connection
```bash
# Check if teleop is publishing
ros2 topic hz /cmd_vel

# Echo the messages
ros2 topic echo /cmd_vel

# Check the full ROS graph
rqt_graph
```

Expected graph:
```
keyboard_teleop_node → /cmd_vel → esp32_bridge_node
                                     ↓
                                  /odom, /imu
```

## Troubleshooting

### Terminal Input Not Working
**Issue:** Keys don't control robot, or terminal shows characters instead

**Solution:**
- Make sure you're running the node in the active terminal (not in background)
- Don't redirect output or use launch files without `prefix='xterm -e'`
- If using tmux/screen, try running in a native terminal

### Robot Doesn't Stop When Releasing Keys
**Issue:** Robot continues moving after releasing key

**Explanation:** This is by design! The node implements a "dead man's switch":
- If no key is pressed for >0.1 seconds, it sends (0, 0) to stop the robot
- This is a safety feature

**To test:**
```bash
# Monitor cmd_vel in another terminal
ros2 topic echo /cmd_vel
# Press W, then release - you should see velocity go to zero
```

### Speed Too Fast/Slow
**Issue:** Default speeds don't match your robot

**Solution 1:** Adjust at runtime with +/- keys

**Solution 2:** Launch with custom parameters:
```bash
ros2 run my_robot_bringup keyboard_teleop \
  --ros-args \
  -p linear_speed_step:=0.05 \
  -p angular_speed_step:=0.3
```

**Solution 3:** Edit default values in the node code:
```python
self.declare_parameter('linear_speed_step', 0.05)   # Change from 0.1
self.declare_parameter('angular_speed_step', 0.3)   # Change from 0.5
```

### Permission Denied (Terminal)
**Issue:** `termios` error on startup

**Solution:**
- Ensure you're running in a proper terminal (not via SSH without TTY)
- Try: `python3 -c "import termios; import sys; termios.tcgetattr(sys.stdin)"`

### Robot Moves Erratically
**Issue:** Robot vibrates or moves unexpectedly

**Check:**
1. ESP32 serial connection quality
2. Motor driver calibration
3. Velocity scaling in `esp32_bridge_node.py`:
   ```python
   speed_left = int(v_left * 100)  # ← Adjust this multiplier
   ```

## Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed_step` | 0.1 | Base linear velocity (m/s) |
| `angular_speed_step` | 0.5 | Base angular velocity (rad/s) |
| `max_linear_speed` | 0.5 | Maximum linear velocity (m/s) |
| `max_angular_speed` | 2.0 | Maximum angular velocity (rad/s) |
| `speed_increment` | 0.05 | Linear increment per +/- press |
| `angular_increment` | 0.25 | Angular increment per +/- press |

## Advanced Usage

### Record Teleoperation Session
```bash
# Start recording
ros2 bag record /cmd_vel /odom /imu

# Teleop your robot
# Ctrl+C to stop recording

# Replay later
ros2 bag play <bag_file>
```

### Remap Topics
```bash
ros2 run my_robot_bringup keyboard_teleop \
  --ros-args \
  --remap /cmd_vel:=/my_robot/cmd_vel
```

### Multiple Robots
```bash
# Robot 1
ros2 run my_robot_bringup keyboard_teleop \
  --ros-args \
  --remap /cmd_vel:=/robot1/cmd_vel

# Robot 2  
ros2 run my_robot_bringup keyboard_teleop \
  --ros-args \
  --remap /cmd_vel:=/robot2/cmd_vel
```

## Safety Features

1. **Dead Man's Switch:** Robot stops if no key pressed for 0.1 seconds
2. **Emergency Stop:** SPACE or X immediately sends (0, 0)
3. **Speed Limits:** Enforces max_linear_speed and max_angular_speed
4. **Graceful Shutdown:** Ctrl+C sends stop command before exiting
5. **Terminal Restoration:** Always restores terminal settings on exit

## Integration Checklist

- [ ] File copied to package directory
- [ ] `setup.py` updated with entry point
- [ ] Package built with `colcon build`
- [ ] ESP32 bridge node running
- [ ] Serial connection established (`/dev/ttyUSB0`)
- [ ] Teleop node starts without errors
- [ ] Control menu displays correctly
- [ ] Keys control robot movement
- [ ] Emergency stop works (SPACE)
- [ ] Speed adjustment works (+/-)
- [ ] Robot stops when keys released
- [ ] Ctrl+C cleanly exits and stops robot
