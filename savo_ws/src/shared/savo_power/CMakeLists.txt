cmake_minimum_required(VERSION 3.16)
project(savo_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

set(SAVO_POWER_DEPENDENCIES
  rclcpp
  std_msgs
  diagnostic_msgs
  savo_msgs
)

set(SAVO_POWER_LIBRARY_SOURCES
  src/state/power_state.cpp
  src/policy/power_policy.cpp
  src/drivers/linux_i2c_bus.cpp
  src/drivers/ups_hat_driver.cpp
  src/drivers/ads7830_driver.cpp
  src/aggregation/power_aggregator.cpp
  src/health/power_health.cpp
  src/nodes/ups_hat_node.cpp
  src/nodes/kit_battery_node.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SAVO_POWER_LIBRARY_SOURCES}
)

target_compile_features(${PROJECT_NAME}
  PUBLIC cxx_std_17
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${SAVO_POWER_DEPENDENCIES}
)

function(savo_power_add_node target_name source_file)
  add_executable(${target_name}
    ${source_file}
  )

  target_compile_features(${target_name}
    PRIVATE cxx_std_17
  )

  target_link_libraries(${target_name}
    ${PROJECT_NAME}
  )

  ament_target_dependencies(${target_name}
    ${SAVO_POWER_DEPENDENCIES}
  )

  list(APPEND SAVO_POWER_NODE_TARGETS ${target_name})
  set(SAVO_POWER_NODE_TARGETS "${SAVO_POWER_NODE_TARGETS}" PARENT_SCOPE)
endfunction()

savo_power_add_node(core_ups_node src/nodes/core_ups_node.cpp)
savo_power_add_node(edge_ups_node src/nodes/edge_ups_node.cpp)
savo_power_add_node(base_battery_node src/nodes/base_battery_node.cpp)
savo_power_add_node(power_aggregator_node src/nodes/power_aggregator_node.cpp)
savo_power_add_node(power_health_node src/nodes/power_health_node.cpp)
savo_power_add_node(power_dashboard_node src/nodes/power_dashboard_node.cpp)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

foreach(dir config launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

file(GLOB SAVO_POWER_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*"
)

if(SAVO_POWER_SCRIPTS)
  install(
    PROGRAMS ${SAVO_POWER_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${SAVO_POWER_NODE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_export_dependencies(
  ${SAVO_POWER_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB SAVO_POWER_PYTEST_FILES test/test_*.py)

  foreach(test_file ${SAVO_POWER_PYTEST_FILES})
    get_filename_component(test_name ${test_file} NAME_WE)
    ament_add_pytest_test(${test_name} ${test_file}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
      TIMEOUT 90
    )
  endforeach()
endif()

ament_package()
