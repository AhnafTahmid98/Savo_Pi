cmake_minimum_required(VERSION 3.8)
project(savo_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(savo_realsense_depth_front_min
  src/depth_front_min_node.cpp
)

target_include_directories(savo_realsense_depth_front_min
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(savo_realsense_depth_front_min
  rclcpp
  sensor_msgs
  std_msgs
)

add_library(savo_realsense_camera_monitor_common
  src/camera_monitor_common.cpp
)

target_include_directories(savo_realsense_camera_monitor_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(savo_realsense_camera_monitor_common
  rclcpp
  diagnostic_msgs
)

add_executable(depth_front_min_node
  src/depth_front_min_main.cpp
)

target_include_directories(depth_front_min_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(depth_front_min_node
  savo_realsense_depth_front_min
)

ament_target_dependencies(depth_front_min_node
  rclcpp
)

add_executable(camera_health_node
  src/camera_health_main.cpp
)

target_include_directories(camera_health_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(camera_health_node
  savo_realsense_camera_monitor_common
)

ament_target_dependencies(camera_health_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
)

add_executable(camera_topic_monitor_node
  src/camera_topic_monitor_main.cpp
)

target_include_directories(camera_topic_monitor_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(camera_topic_monitor_node
  savo_realsense_camera_monitor_common
)

ament_target_dependencies(camera_topic_monitor_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

install(TARGETS
  savo_realsense_depth_front_min
  savo_realsense_camera_monitor_common
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  depth_front_min_node
  camera_health_node
  camera_topic_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/camera_health_node_py
  scripts/camera_topic_monitor_node_py
  scripts/depth_front_min_node_py
  scripts/realsense_smoke_test_cli
  scripts/dump_effective_realsense_params
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES
    README.md
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB PYTEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "test/test_*.py")

  foreach(test_path ${PYTEST_FILES})
    get_filename_component(test_name "${test_path}" NAME_WE)
    ament_add_pytest_test(${test_name} "${test_path}")
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  savo_realsense_depth_front_min
  savo_realsense_camera_monitor_common
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
