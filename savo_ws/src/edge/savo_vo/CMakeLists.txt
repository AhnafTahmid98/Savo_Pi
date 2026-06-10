cmake_minimum_required(VERSION 3.8)
project(savo_vo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/rgbd_odometry_node
    scripts/vo_health_node
    scripts/vo_diagnostics_node
    scripts/vo_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_imports test/test_imports.py)
  ament_add_pytest_test(test_topic_names test/test_topic_names.py)
  ament_add_pytest_test(test_frame_names test/test_frame_names.py)
  ament_add_pytest_test(test_parameter_names test/test_parameter_names.py)
  ament_add_pytest_test(test_vo_status test/test_vo_status.py)
  ament_add_pytest_test(test_tracking_report test/test_tracking_report.py)
  ament_add_pytest_test(test_timestamp_sync test/test_timestamp_sync.py)
  ament_add_pytest_test(test_tracking_quality test/test_tracking_quality.py)
  ament_add_pytest_test(test_motion_quality test/test_motion_quality.py)
  ament_add_pytest_test(test_odom_checks test/test_odom_checks.py)
  ament_add_pytest_test(test_covariance_builder test/test_covariance_builder.py)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
endif()

ament_package()
