cmake_minimum_required(VERSION 3.8)
project(savo_vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d)

ament_python_install_package(${PROJECT_NAME})

add_library(savo_vo_core
  src/vo_types.cpp
  src/tracking_quality.cpp
  src/timestamp_sync.cpp
  src/covariance_builder.cpp
  src/geometry_utils.cpp
  src/rgbd_odometry_node.cpp
  src/vo_health_node.cpp
  src/vo_diagnostics_node.cpp
  src/vo_republisher_node.cpp
)

target_include_directories(savo_vo_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(savo_vo_core
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(savo_vo_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  builtin_interfaces
  cv_bridge
)

target_link_libraries(savo_vo_core
  ${OpenCV_LIBS}
)

add_executable(rgbd_odometry_node
  src/rgbd_odometry_main.cpp
)

target_link_libraries(rgbd_odometry_node
  savo_vo_core
)

ament_target_dependencies(rgbd_odometry_node
  rclcpp
)

add_executable(vo_health_node
  src/vo_health_main.cpp
)

target_link_libraries(vo_health_node
  savo_vo_core
)

ament_target_dependencies(vo_health_node
  rclcpp
)

add_executable(vo_diagnostics_node
  src/vo_diagnostics_main.cpp
)

target_link_libraries(vo_diagnostics_node
  savo_vo_core
)

ament_target_dependencies(vo_diagnostics_node
  rclcpp
)

add_executable(vo_republisher_node
  src/vo_republisher_main.cpp
)

target_link_libraries(vo_republisher_node
  savo_vo_core
)

ament_target_dependencies(vo_republisher_node
  rclcpp
)

install(
  TARGETS
    savo_vo_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    rgbd_odometry_node
    vo_health_node
    vo_diagnostics_node
    vo_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/rgbd_odometry_node_py
    scripts/vo_health_node_py
    scripts/vo_diagnostics_node_py
    scripts/vo_republisher_node_py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(
    FILES README.md
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB PYTEST_FILES
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "test/test_*.py"
  )

  foreach(test_path ${PYTEST_FILES})
    get_filename_component(test_name "${test_path}" NAME_WE)
    ament_add_pytest_test(${test_name} "${test_path}")
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(savo_vo_core)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
