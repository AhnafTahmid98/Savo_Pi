cmake_minimum_required(VERSION 3.8)
project(savo_base)

# -----------------------------
# Compiler defaults (C++)
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Core ROS 2 (Python + optional C++)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Messages
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

# Launch/runtime helper package used by Python launch/config lookup
find_package(ament_index_python REQUIRED)

# -----------------------------
# Python package install
# -----------------------------
# Installs the Python package directory:
#   savo_base/
# so imports like "from savo_base.drivers ... " work.
ament_python_install_package(${PROJECT_NAME})

# -----------------------------
# Install Python executables (scripts/ and ROS nodes if executed directly)
# -----------------------------
# Keep this list explicit and stable for professional package control.
# Add files here as you implement them.
install(PROGRAMS
  scripts/teleop_letters_cli.py
  scripts/poke_test_cli.py
  scripts/base_smoke_test_cli.py
  scripts/dump_effective_params.py

  savo_base/nodes/base_driver_node.py
  savo_base/nodes/base_watchdog_node.py
  savo_base/nodes/base_state_publisher_node.py
  savo_base/nodes/base_diag_runner_node.py
  savo_base/nodes/base_heartbeat_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Install non-code resources
# -----------------------------
install(DIRECTORY
  launch
  config
  resource
  DESTINATION share/${PROJECT_NAME}
)

# README is useful in install tree for deployed robot docs/troubleshooting
install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------
# Optional C++ targets (future-ready scaffold)
# -----------------------------
# Add C++ nodes/libraries here later if needed, e.g.:
#
# add_executable(example_cpp_node src/example_cpp_node.cpp)
# ament_target_dependencies(example_cpp_node
#   rclcpp
#   std_msgs
# )
# install(TARGETS
#   example_cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# -----------------------------
# Testing / Lint
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Python tests via pytest (works with ament_cmake + ament_python hybrid)
  find_package(ament_cmake_pytest REQUIRED)

  # Add tests only if files exist (safe while scaffolding)
  ament_add_pytest_test(test_mecanum_mix test/test_mecanum_mix.py)
  ament_add_pytest_test(test_wheel_command test/test_wheel_command.py)
  ament_add_pytest_test(test_clamp_and_scaling test/test_clamp_and_scaling.py)
  ament_add_pytest_test(test_timeout_watchdog_logic test/test_timeout_watchdog_logic.py)
  ament_add_pytest_test(test_topic_names test/test_topic_names.py)
  ament_add_pytest_test(test_param_loader test/test_param_loader.py)
  ament_add_pytest_test(test_driver_dryrun test/test_driver_dryrun.py)
endif()

# -----------------------------
# Package export
# -----------------------------
ament_package()