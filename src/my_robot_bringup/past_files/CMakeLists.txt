cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= DEPENDENCIES =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ================= C++ EXECUTABLES =================
add_executable(encoder_odom_node src/encoder_odom_node.cpp)
ament_target_dependencies(encoder_odom_node 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  tf2 
  tf2_ros
)

# ================= INSTALLATION =================

# 1. Install C++ Nodes
install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install Folders (تم تصحيح الخطأ المطبعي هنا)
install(DIRECTORY
  launch
  
  
  config    # هذا المجلد يحتوي على ملف ekf.yaml
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install Python Scripts
install(PROGRAMS
  src/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
