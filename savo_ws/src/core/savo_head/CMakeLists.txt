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
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/core/head_state.cpp
  src/core/scan_pattern.cpp
  src/core/servo_calibration.cpp
  src/core/head_config.cpp
  src/core/head_status.cpp
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
    diagnostic_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
  )

  install(TARGETS ${target_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_savo_head_node(head_controller_node src/nodes/head_controller_node.cpp)
add_savo_head_node(head_scan_node src/nodes/head_scan_node.cpp)
add_savo_head_node(head_tf_node src/nodes/head_tf_node.cpp)
add_savo_head_node(head_status_node src/nodes/head_status_node.cpp)
add_savo_head_node(apriltag_confirm_node src/nodes/apriltag_confirm_node.cpp)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
