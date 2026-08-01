cmake_minimum_required(VERSION 3.16)
project(savo_head VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(savo_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED IMPORTED_TARGET apriltag)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}_core
  src/core/head_state.cpp
  src/core/scan_pattern.cpp
  src/core/servo_calibration.cpp
  src/core/head_config.cpp
  src/core/head_status.cpp
  src/core/camera_health.cpp
  src/core/camera_status_report.cpp
  src/core/apriltag_detector_config.cpp
  src/drivers/pca9685_driver.cpp
  src/drivers/pantilt_driver.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_core
)

function(add_savo_head_node target_name source_file)
  add_executable(${target_name} ${source_file})

  target_include_directories(${target_name}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${target_name}
    ${PROJECT_NAME}_core
  )

  ament_target_dependencies(${target_name}
    rclcpp
    rclcpp_action
    savo_msgs
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(TARGETS ${target_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_savo_head_node(head_controller_node src/nodes/head_controller_node.cpp)
add_savo_head_node(head_scan_node src/nodes/head_scan_node.cpp)
add_savo_head_node(head_tf_node src/nodes/head_tf_node.cpp)
add_savo_head_node(head_status_node src/nodes/head_status_node.cpp)
add_savo_head_node(head_camera_status_node src/nodes/head_camera_status_node.cpp)
add_savo_head_node(apriltag_confirm_node src/nodes/apriltag_confirm_node.cpp)
add_savo_head_node(
  apriltag_confirmation_action_node
  src/nodes/apriltag_confirmation_action_node.cpp
)

add_executable(
  apriltag_detector_node
  src/nodes/apriltag_detector_node.cpp
)

target_include_directories(apriltag_detector_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(apriltag_detector_node
  ${PROJECT_NAME}_core
  PkgConfig::APRILTAG
  ${OpenCV_LIBS}
  Threads::Threads
)

ament_target_dependencies(apriltag_detector_node
  rclcpp
  savo_msgs
  diagnostic_msgs
  sensor_msgs
  cv_bridge
  tf2
)

install(TARGETS apriltag_detector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

install(
  PROGRAMS
    scripts/head_camera_view.py
    scripts/head_manual_cli.py
    scripts/apriltag_debug_cli.py
    scripts/dump_effective_head_params.py
    scripts/head_controller_node_py
    scripts/head_scan_node_py
    scripts/head_tf_node_py
    scripts/head_status_node_py
    scripts/apriltag_confirm_node_py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

option(
  SAVO_HEAD_ENABLE_LEGACY_LINT
  "Run package-wide legacy lint tests"
  OFF
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_camera_health_core
    test/test_camera_health_core.py
  )

  ament_add_pytest_test(
    test_camera_status_node_contract
    test/test_camera_status_node_contract.py
  )

  ament_add_pytest_test(
    test_camera_status_report_core
    test/test_camera_status_report_core.py
  )

  ament_add_pytest_test(
    test_head_status_camera_severity_contract
    test/test_head_status_camera_severity_contract.py
  )

  ament_add_pytest_test(
    test_camera_launch_integration
    test/test_camera_launch_integration.py
  )

  ament_add_pytest_test(
    test_controller_diagnostic_ownership
    test/test_controller_diagnostic_ownership.py
  )

  ament_add_pytest_test(
    test_apriltag_typed_contract
    test/test_apriltag_typed_contract.py
  )

  ament_add_pytest_test(
    test_apriltag_detector_config_core
    test/test_apriltag_detector_config_core.py
  )

  ament_add_pytest_test(
    test_apriltag_detector_node_contract
    test/test_apriltag_detector_node_contract.py
  )

  ament_add_pytest_test(
    test_apriltag_action_runtime_contract
    test/test_apriltag_action_runtime_contract.py
  )

  ament_add_pytest_test(
    test_apriltag_production_launch_contract
    test/test_apriltag_production_launch_contract.py
  )

  ament_add_pytest_test(
    test_head_tf_calibration_gate_contract
    test/test_head_tf_calibration_gate_contract.py
  )

  if(SAVO_HEAD_ENABLE_LEGACY_LINT)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
