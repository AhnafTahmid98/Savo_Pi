# CMakeLists.txt
# Robot SAVO — savo_localization (ROS 2 Jazzy)
#
# Hybrid package (ament_cmake + ament_python):
# - C++: wheel_odom_node (production) built with ament_cmake
# - Python: imu_node + fallback node + sensors_api installed with ament_python
# - Installs: launch/, config/, include/, and ament resource marker
#
# NOTE:
# - wheel_odom_node uses liblgpio (C API). We must link against -llgpio.

cmake_minimum_required(VERSION 3.8)
project(savo_localization)

# -----------------------
# Build settings
# -----------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optional but recommended
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

# ---- Link liblgpio (required for lgGpio* symbols) ----
find_library(LGPIO_LIB lgpio)
if(NOT LGPIO_LIB)
  message(FATAL_ERROR "lgpio library not found. Install with: sudo apt install -y liblgpio-dev")
endif()
target_link_libraries(wheel_odom_node ${LGPIO_LIB})

install(TARGETS wheel_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Python package install
# -----------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------
# Install Python node entry scripts (ros2 run)
# -----------------------
install(PROGRAMS
  scripts/imu_node
  scripts/wheel_odom_fallback_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Install headers
# -----------------------
install(
  DIRECTORY include/
  DESTINATION include/
)

# -----------------------
# Install launch + config
# -----------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------
# Install resource marker
# -----------------------
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()