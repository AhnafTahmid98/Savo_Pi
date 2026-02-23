cmake_minimum_required(VERSION 3.8)
project(savo_base)

# -----------------------------------------------------------------------------
# Compiler defaults (C++)
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # provides ament_python_install_package()

# Core ROS 2 (Python + optional C++)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Messages
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

# NOTE:
# ament_index_python is a Python runtime package, not a CMake package.
# Keep it in package.xml / Python imports, but DO NOT find_package() it here.

# -----------------------------------------------------------------------------
# Python package install
# -----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------------------------------------------------------
# Install Python executables
# -----------------------------------------------------------------------------
# Hybrid-package strategy:
# - Install ROS node Python files explicitly (reliable for ament_cmake hybrids)
# - Install CLI/helper scripts explicitly
install(PROGRAMS
  # CLI tools (scripts/)
  scripts/teleop_letters_cli.py
  scripts/poke_test_cli.py
  scripts/base_smoke_test_cli.py
  scripts/dump_effective_params.py

  # ROS nodes (savo_base/nodes/)
  savo_base/nodes/base_driver_node.py
  savo_base/nodes/base_watchdog_node.py
  savo_base/nodes/base_state_publisher_node.py
  savo_base/nodes/base_diag_runner_node.py
  savo_base/nodes/base_heartbeat_node.py

  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Install non-code resources
# -----------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  resource
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Optional C++ targets (future-ready scaffold)
# -----------------------------------------------------------------------------
# Add C++ nodes/libraries here later if needed.

# -----------------------------------------------------------------------------
# Testing / Lint
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_mecanum_mix.py")
    ament_add_pytest_test(test_mecanum_mix test/test_mecanum_mix.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_wheel_command.py")
    ament_add_pytest_test(test_wheel_command test/test_wheel_command.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_clamp_and_scaling.py")
    ament_add_pytest_test(test_clamp_and_scaling test/test_clamp_and_scaling.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_timeout_watchdog_logic.py")
    ament_add_pytest_test(test_timeout_watchdog_logic test/test_timeout_watchdog_logic.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_topic_names.py")
    ament_add_pytest_test(test_topic_names test/test_topic_names.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_param_loader.py")
    ament_add_pytest_test(test_param_loader test/test_param_loader.py)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_driver_dryrun.py")
    ament_add_pytest_test(test_driver_dryrun test/test_driver_dryrun.py)
  endif()
endif()

# -----------------------------------------------------------------------------
# Package export
# -----------------------------------------------------------------------------
ament_package()