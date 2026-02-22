cmake_minimum_required(VERSION 3.8)
project(savo_control)

# ============================================================
# Robot SAVO — savo_control (ROS 2 Jazzy)
# Hybrid package:
#   - C++ nodes: mux, shaper, PID/heading, recovery, mode manager
#   - Python nodes: test managers, dashboards, status helpers (optional/future)
#
# Rationale:
# - In this repo style, relying only on setup.py entry_points has not always
#   produced reliable ROS executables in install/lib/<pkg>.
# - We therefore install Python ROS nodes explicitly via CMake install(PROGRAMS)
#   for reliable `ros2 run`.
# - Python package modules/libraries are still installed via
#   ament_python_install_package().
#
# Notes
# -----
# - This CMakeLists matches the organized src layout:
#     src/controllers/
#     src/filters/
#     src/utils/
#     src/recovery/
#     src/nodes/
# - Header-only utilities still get placeholder .cpp files so the core library
#   can keep a stable structure and link target.
# ============================================================

# -----------------------
# Compiler settings
# -----------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------
# Explicit Python package name
# -----------------------
set(PYTHON_PACKAGE_NAME "savo_control")

# -----------------------
# Dependencies
# -----------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# TF / transforms (for heading / rotate controllers)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Project messages (optional today, useful later)
find_package(savo_msgs REQUIRED)

# -----------------------
# Include dirs
# -----------------------
# Global include is acceptable here; targets also set explicit includes below.
include_directories(include)

# -----------------------
# C++ library: core control utilities
# -----------------------
# Keep reusable logic in one library so nodes stay thin and professional.
add_library(savo_control_core
  # Controllers
  src/controllers/pid.cpp
  src/controllers/heading_controller.cpp

  # Filters / shaping
  src/filters/command_limiter.cpp

  # Utils
  src/utils/control_math.cpp

  # Recovery helpers
  src/recovery/recovery_manager.cpp
  src/recovery/stuck_detector.cpp
)

target_include_directories(savo_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Keep this library ROS-friendly. Some files are header-only today, but linking
# ROS deps here is fine and simplifies downstream node targets.
ament_target_dependencies(savo_control_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  savo_msgs
)

# -----------------------
# Helper function for C++ ROS nodes
# -----------------------
function(savo_add_node target source_file)
  add_executable(${target} ${source_file})

  target_link_libraries(${target}
    savo_control_core
  )

  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    savo_msgs
  )

  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# -----------------------
# C++ nodes (organized under src/nodes/)
# -----------------------
# Core command path
savo_add_node(twist_mux_node                 src/nodes/twist_mux_node.cpp)
savo_add_node(cmd_vel_shaper_node            src/nodes/cmd_vel_shaper_node.cpp)
savo_add_node(control_mode_manager_node      src/nodes/control_mode_manager_node.cpp)

# Motion / heading / test controllers
savo_add_node(heading_pid_node               src/nodes/heading_pid_node.cpp)
savo_add_node(rotate_to_heading_node         src/nodes/rotate_to_heading_node.cpp)
savo_add_node(velocity_test_pattern_node     src/nodes/velocity_test_pattern_node.cpp)

# Recovery path
savo_add_node(recovery_manager_node          src/nodes/recovery_manager_node.cpp)
savo_add_node(backup_escape_node             src/nodes/backup_escape_node.cpp)

# -----------------------
# Python package install (modules/imports)
# -----------------------
# Keeps Python package importable (e.g., savo_control.* helpers)
ament_python_install_package(${PYTHON_PACKAGE_NAME})

# -----------------------
# Install Python ROS executable nodes explicitly
# (reliable `ros2 run` in hybrid package)
# -----------------------
# Keep only files that actually exist in your current layout.
# Add/remove entries as you create Python nodes.
set(SAVO_CONTROL_PY_NODES
  savo_control/nodes/auto_test_manager_node.py
  savo_control/nodes/straight_line_pid_test_node.py
  savo_control/nodes/distance_pid_test_node.py
  savo_control/nodes/control_status_node.py
  savo_control/nodes/control_dashboard_node.py
  savo_control/nodes/recovery_test_manager_node.py
  savo_control/nodes/recovery_status_node.py
)

set(SAVO_CONTROL_PY_NODES_EXISTING "")
foreach(py_node IN LISTS SAVO_CONTROL_PY_NODES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${py_node}")
    list(APPEND SAVO_CONTROL_PY_NODES_EXISTING "${py_node}")
  endif()
endforeach()

if(SAVO_CONTROL_PY_NODES_EXISTING)
  install(PROGRAMS
    ${SAVO_CONTROL_PY_NODES_EXISTING}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
# Install docs folder (optional)
# -----------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  install(
    DIRECTORY docs/
    DESTINATION share/${PROJECT_NAME}/docs
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
# Tests / lint
# -----------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------
# Export
# -----------------------
ament_export_include_directories(include)
ament_export_libraries(savo_control_core)

# -----------------------
# Package
# -----------------------
ament_package()