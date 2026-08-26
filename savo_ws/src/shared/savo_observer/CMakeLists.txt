cmake_minimum_required(VERSION 3.16)
project(savo_observer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(savo_observer_core SHARED
  src/observer_contract.cpp
  src/telemetry_freshness.cpp
  src/telemetry_snapshot.cpp
)
target_include_directories(savo_observer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(observer_telemetry_node src/nodes/observer_telemetry_node.cpp)
target_link_libraries(observer_telemetry_node savo_observer_core)
ament_target_dependencies(observer_telemetry_node
  diagnostic_msgs
  rclcpp
  std_msgs
)

add_executable(observer_dashboard_node src/nodes/observer_dashboard_node.cpp)
ament_target_dependencies(observer_dashboard_node
  ament_index_cpp
  rclcpp
  std_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config dashboard launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES LICENSE package.xml README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/check_observer_connection
  scripts/list_observer_views
  scripts/validate_observer_install
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  savo_observer_core
  observer_telemetry_node
  observer_dashboard_node
  EXPORT export_savo_observer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_savo_observer HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  foreach(test_name
      test_observer_contract
      test_telemetry_freshness
      test_telemetry_snapshot)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} savo_observer_core)
    target_include_directories(${test_name} PRIVATE include)
  endforeach()

  file(GLOB observer_pytests test/test_*.py)
  foreach(test_file ${observer_pytests})
    get_filename_component(test_name ${test_file} NAME_WE)
    ament_add_pytest_test(${test_name} ${test_file}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      TIMEOUT 120
    )
  endforeach()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
