cmake_minimum_required(VERSION 3.8)
project(savo_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Python package
# ---------------------------------------------------------------------------

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/dump_effective_params.py
    scripts/find_lidar_port_cli.py
    scripts/lidar_smoke_test_cli.py
    scripts/scan_echo_cli.py
    scripts/scan_quality_cli.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    savo_lidar/nodes/lidar_py_driver_node.py
    savo_lidar/nodes/lidar_filter_node.py
    savo_lidar/nodes/lidar_watchdog_node.py
    savo_lidar/nodes/lidar_health_node.py
    savo_lidar/nodes/lidar_state_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# C++ library
# ---------------------------------------------------------------------------

add_library(${PROJECT_NAME}_cpp SHARED
  src/drivers/serial_port.cpp
  src/drivers/rplidar_protocol.cpp
  src/drivers/rplidar_driver.cpp
  src/ros/scan_publisher.cpp
  src/ros/diagnostics.cpp
)

target_include_directories(${PROJECT_NAME}_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_cpp PUBLIC cxx_std_17)

target_compile_definitions(${PROJECT_NAME}_cpp
  PRIVATE SAVO_LIDAR_BUILDING_LIBRARY
)

ament_target_dependencies(${PROJECT_NAME}_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

# ---------------------------------------------------------------------------
# C++ nodes
# ---------------------------------------------------------------------------

add_executable(lidar_driver_node
  src/nodes/lidar_driver_node.cpp
)

target_compile_features(lidar_driver_node PUBLIC cxx_std_17)

target_include_directories(lidar_driver_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lidar_driver_node
  ${PROJECT_NAME}_cpp
)

ament_target_dependencies(lidar_driver_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ---------------------------------------------------------------------------
# Install C++ targets and resources
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_cpp
  EXPORT export_${PROJECT_NAME}_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    lidar_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Temporarily disabled while the production C++ layer and launch files are
  # being staged. Core pytest and build checks remain active.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_angle_filter test/test_angle_filter.py)
  ament_add_pytest_test(test_range_filter test/test_range_filter.py)
  ament_add_pytest_test(test_scan_rate_filter test/test_scan_rate_filter.py)
  ament_add_pytest_test(test_sector_extractor test/test_sector_extractor.py)
  ament_add_pytest_test(test_scan_watchdog test/test_scan_watchdog.py)
  ament_add_pytest_test(test_stale_scan_policy test/test_stale_scan_policy.py)
  ament_add_pytest_test(test_topic_names test/test_topic_names.py)
  ament_add_pytest_test(test_param_loader test/test_param_loader.py)
  ament_add_pytest_test(test_driver_dryrun test/test_driver_dryrun.py)
  ament_add_pytest_test(test_lidar_models test/test_lidar_models.py)
endif()

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_cpp HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
)

ament_package()