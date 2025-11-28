cmake_minimum_required(VERSION 3.8)
project(savo_ui)

# ============================================================================
# Robot Savo — UI stack (pure ament_python)
#
# Build & install are driven by setup.py / setup.cfg.
# This CMakeLists is intentionally minimal: it just hooks into ament so
# colcon knows this package exists, and lets setuptools do the work.
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (savo_ui) from setup.py / setup.cfg
ament_python_install_package(${PROJECT_NAME})

# Non-Python resources (launch, config, resource) are also installed via
# setup.py's data_files, but we mirror them here for ROS 2 tools.
install(
  DIRECTORY launch config resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
