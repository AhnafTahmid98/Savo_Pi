cmake_minimum_required(VERSION 3.8)
project(savo_perception)

# -----------------------------
# Compiler settings
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# -----------------------------
# C++: cmd_vel_safety_gate node
# -----------------------------
add_executable(cmd_vel_safety_gate
  src/cmd_vel_safety_gate.cpp
)

target_include_directories(cmd_vel_safety_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cmd_vel_safety_gate
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS cmd_vel_safety_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# -----------------------------
# Python package (rclpy nodes + sensors_api)
# -----------------------------
ament_python_install_package(${PROJECT_NAME})

# Install Python node scripts as ROS executables for `ros2 run`
# NOTE: We "RENAME" to remove .py so the command is clean:
#   ros2 run savo_perception vl53_node
install(PROGRAMS
  savo_perception/nodes/vl53_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vl53_node
)

install(PROGRAMS
  savo_perception/nodes/ultrasonic_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ultrasonic_node
)

install(PROGRAMS
  savo_perception/nodes/depth_front_min_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_front_min_node
)

install(PROGRAMS
  savo_perception/nodes/safety_stop_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME safety_stop_node
)

# NEW: terminal dashboard (curses)
install(PROGRAMS
  savo_perception/nodes/sensor_dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_dashboard
)

# Install launch + config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Tests / lint
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
