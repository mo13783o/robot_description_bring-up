#!/bin/bash

###############################################################################
# Robot System Quick Start Script
# This script helps verify that all components are properly installed
###############################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================================================="
echo "          ROBOT SYSTEM VERIFICATION AND QUICK START"
echo "========================================================================="
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if ROS package exists
package_exists() {
    ros2 pkg prefix "$1" >/dev/null 2>&1
}

echo "Checking dependencies..."
echo ""

# Check ROS 2 installation
if ! command_exists ros2; then
    echo -e "${RED}✗ ROS 2 not found${NC}"
    echo "  Please install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
else
    echo -e "${GREEN}✓ ROS 2 found${NC}"
fi

# Check Python
if ! command_exists python3; then
    echo -e "${RED}✗ Python 3 not found${NC}"
    exit 1
else
    echo -e "${GREEN}✓ Python 3 found${NC}"
fi

# Check pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    echo -e "${YELLOW}⚠ pyserial not found${NC}"
    echo "  Installing pyserial..."
    pip3 install pyserial --break-system-packages
else
    echo -e "${GREEN}✓ pyserial found${NC}"
fi

# Check xterm (for teleop)
if ! command_exists xterm; then
    echo -e "${YELLOW}⚠ xterm not found (needed for keyboard teleop)${NC}"
    echo "  Install with: sudo apt install xterm"
else
    echo -e "${GREEN}✓ xterm found${NC}"
fi

echo ""
echo "Checking ROS 2 packages..."
echo ""

# Check robot_localization
if ! package_exists robot_localization; then
    echo -e "${YELLOW}⚠ robot_localization not found${NC}"
    echo "  Install with: sudo apt install ros-humble-robot-localization"
else
    echo -e "${GREEN}✓ robot_localization found${NC}"
fi

# Check my_robot_description
if ! package_exists my_robot_description; then
    echo -e "${YELLOW}⚠ my_robot_description not found${NC}"
    echo "  Please build this package first"
else
    echo -e "${GREEN}✓ my_robot_description found${NC}"
fi

# Check my_robot_bringup
if ! package_exists my_robot_bringup; then
    echo -e "${RED}✗ my_robot_bringup not found${NC}"
    echo "  This package needs to be built!"
    echo ""
    echo "  Run:"
    echo "    cd ~/ros2_ws"
    echo "    colcon build --packages-select my_robot_bringup"
    echo "    source install/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ my_robot_bringup found${NC}"
fi

echo ""
echo "Checking serial port..."
echo ""

# Check for serial ports
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✓ USB serial port(s) found:${NC}"
    ls /dev/ttyUSB*
elif ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✓ ACM serial port(s) found:${NC}"
    ls /dev/ttyACM*
else
    echo -e "${YELLOW}⚠ No serial ports found${NC}"
    echo "  Make sure your ESP32 is connected"
fi

# Check serial port permissions
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    PORT=$(ls /dev/ttyUSB* | head -n 1)
elif ls /dev/ttyACM* 1> /dev/null 2>&1; then
    PORT=$(ls /dev/ttyACM* | head -n 1)
else
    PORT=""
fi

if [ -n "$PORT" ]; then
    if [ -r "$PORT" ] && [ -w "$PORT" ]; then
        echo -e "${GREEN}✓ Serial port $PORT is accessible${NC}"
    else
        echo -e "${YELLOW}⚠ No permission to access $PORT${NC}"
        echo "  Run: sudo usermod -a -G dialout $USER"
        echo "  Then log out and back in"
    fi
fi

echo ""
echo "========================================================================="
echo "                    VERIFICATION COMPLETE"
echo "========================================================================="
echo ""
echo "Available launch options:"
echo ""
echo "1. Full system (ESP32 + EKF + Teleop + URDF):"
echo "   ${GREEN}ros2 launch my_robot_bringup robot_full.launch.py${NC}"
echo ""
echo "2. Without keyboard teleop (autonomous mode):"
echo "   ${GREEN}ros2 launch my_robot_bringup robot_full.launch.py run_teleop:=false${NC}"
echo ""
echo "3. Custom serial port:"
echo "   ${GREEN}ros2 launch my_robot_bringup robot_full.launch.py serial_port:=/dev/ttyACM0${NC}"
echo ""
echo "4. ESP32 bridge only:"
echo "   ${GREEN}ros2 launch my_robot_bringup robot_bridge.launch.py${NC}"
echo ""
echo "5. Teleop only (if ESP32 is already running):"
echo "   ${GREEN}ros2 run my_robot_bringup keyboard_teleop${NC}"
echo ""
echo "========================================================================="
echo ""

# Ask user if they want to launch
read -p "Do you want to launch the full system now? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Launching robot system..."
    echo ""
    
    # Source workspace
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source "$HOME/ros2_ws/install/setup.bash"
    fi
    
    # Launch
    ros2 launch my_robot_bringup robot_full.launch.py
else
    echo "Exiting. Run the launch commands above when ready."
fi
