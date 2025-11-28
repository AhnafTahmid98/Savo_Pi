cmake_minimum_required(VERSION 3.8)
project(savo_ui)

# ============================================================================
# Robot Savo — UI stack (ament_python)
#
# - Python-only ROS 2 package (same style as savo_speech)
# - Uses rclpy + pygame to drive the 7" DFRobot DSI display
# - Nodes (Python entry points) live under the savo_ui/ module.
# ============================================================================

# Core ament dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS 2 interfaces used by this package
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

# ----------------------------------------------------------------------------
# Python package installation
#
# This discovers the Python package defined by setup.cfg/setup.py
# (package "savo_ui") and installs it into the overlay.
# ----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# ----------------------------------------------------------------------------
# Install Python node entry points as executables
#
# These are the scripts you will actually run via:
#   ros2 run savo_ui <executable>
#
# Each file must have a proper #!/usr/bin/env python3 shebang and be executable.
# ----------------------------------------------------------------------------
install(
  PROGRAMS
    savo_ui/display_manager_node.py
    savo_ui/ui_mode_router_node.py
    savo_ui/ui_debug_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Install launch files (savo_ui_bringup.launch.py, debug launches, etc.)
# ----------------------------------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Install configuration files (UI layout, colors, camera view settings)
# ----------------------------------------------------------------------------
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# (Optional) install package resource index
# If you add a "resource/savo_ui" file, this makes the package visible to
# `ros2 pkg` and other ament tools.
# ----------------------------------------------------------------------------
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# ament package footer
# ----------------------------------------------------------------------------
ament_package()
