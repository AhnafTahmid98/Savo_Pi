cmake_minimum_required(VERSION 3.8)
project(savo_localization)

# ============================================================
# Robot SAVO — savo_localization (ROS 2 Jazzy)
# Hybrid package:
#   - C++ node: wheel_odom_node (production odometry)
#   - Python nodes: imu_node, wheel_odom_fallback_node, localization_dashboard
#     (installed explicitly via CMake install(PROGRAMS ...))
#
# Rationale:
# - In this package layout, relying only on setup.py entry_points has not
#   consistently produced ROS executables in install/lib/savo_localization.
# - This CMake file installs Python ROS nodes directly for reliable `ros2 run`.
# - Python package modules/libraries are still installed via
#   ament_python_install_package().
# ============================================================

# -----------------------
# Compiler settings
# -----------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------
# Dependencies
# -----------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# -----------------------
# C++ node: wheel_odom_node
# -----------------------
add_executable(wheel_odom_node
  src/wheel_odom_node.cpp
)

target_include_directories(wheel_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wheel_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# -----------------------
# Link lgpio (required)
# -----------------------
find_library(LGPIO_LIB lgpio)
if(NOT LGPIO_LIB)
  message(FATAL_ERROR "lgpio library not found. Install with: sudo apt install -y liblgpio-dev")
endif()

target_link_libraries(wheel_odom_node ${LGPIO_LIB})

# Install C++ executable
install(TARGETS wheel_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Python package install (modules, imports)
# -----------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------
# Install Python ROS executable nodes explicitly
# (reliable `ros2 run` in hybrid package)
# -----------------------
install(PROGRAMS
  savo_localization/nodes/imu_node.py
  savo_localization/nodes/wheel_odom_fallback_node.py
  savo_localization/nodes/localization_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Install headers (if present)
# -----------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include/
  )
endif()

# -----------------------
# Install launch folder (if present)
# -----------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# -----------------------
# Install config folder (if present)
# -----------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# -----------------------
# Install rviz folder (optional)
# -----------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

# -----------------------
# Package
# -----------------------
ament_package()