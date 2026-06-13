cmake_minimum_required(VERSION 3.8)
project(savo_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Python tools and fallback/runtime nodes.
# The production C++ driver will later use the executable name: lidar_driver_node.
install(PROGRAMS
  scripts/lidar_smoke_test_cli.py
  scripts/dump_effective_params.py
  scripts/scan_echo_cli.py
  scripts/scan_quality_cli.py
  scripts/find_lidar_port_cli.py

  savo_lidar/nodes/lidar_driver_node.py
  savo_lidar/nodes/lidar_health_node.py
  savo_lidar/nodes/lidar_filter_node.py
  savo_lidar/nodes/lidar_watchdog_node.py
  savo_lidar/nodes/lidar_state_publisher_node.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY
    include/
    DESTINATION include
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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

ament_package()