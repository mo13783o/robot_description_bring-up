cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

# ============================================================================
# COMPILER SETTINGS
# ============================================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# FIND DEPENDENCIES
# ============================================================================

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_localization REQUIRED)

# ============================================================================
# PYTHON PACKAGE SETUP
# ============================================================================

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# ============================================================================
# INSTALL PYTHON SCRIPTS/NODES
# ============================================================================

# Install Python executables (nodes)
install(PROGRAMS
  ${PROJECT_NAME}/esp32_bridge_node.py
  ${PROJECT_NAME}/keyboard_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Alternative: If your scripts are in a 'scripts' directory
# install(PROGRAMS
#   scripts/esp32_bridge_node.py
#   scripts/keyboard_teleop_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# ============================================================================
# INSTALL LAUNCH FILES
# ============================================================================

# Install launch directory
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# If you have XML launch files as well
# install(DIRECTORY
#   launch/
#   DESTINATION share/${PROJECT_NAME}/launch
#   FILES_MATCHING PATTERN "*.xml"
# )

# ============================================================================
# INSTALL CONFIGURATION FILES
# ============================================================================

# Install config directory (for YAML files like ekf.yaml)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# ============================================================================
# INSTALL RESOURCE FILES (if any)
# ============================================================================

# Install rviz config files (if you have them)
# install(DIRECTORY
#   rviz/
#   DESTINATION share/${PROJECT_NAME}/rviz
#   FILES_MATCHING PATTERN "*.rviz"
# )

# Install parameter files (if you have additional param files)
# install(DIRECTORY
#   params/
#   DESTINATION share/${PROJECT_NAME}/params
#   FILES_MATCHING PATTERN "*.yaml"
# )

# ============================================================================
# INSTALL SRC FILES (if needed for reference)
# ============================================================================

# Optional: Install source files for reference
# This is typically not necessary for Python packages, but can be useful
# install(DIRECTORY
#   src/
#   DESTINATION share/${PROJECT_NAME}/src
#   FILES_MATCHING PATTERN "*.py"
# )

# ============================================================================
# TESTING
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# AMENT PACKAGE
# ============================================================================

ament_package()
