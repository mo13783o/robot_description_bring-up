cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

# ======================================================
# Find dependencies
# ======================================================

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ======================================================
# Install Python executables (nodes)
# ======================================================

install(PROGRAMS
  my_robot_bringup/esp32_bridge_node.py
  my_robot_bringup/keyboard_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ======================================================
# Install launch files
# ======================================================

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ======================================================
# Install config files
# ======================================================

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# ======================================================
# Ament package
# ======================================================

ament_package()
