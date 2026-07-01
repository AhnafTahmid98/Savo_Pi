cmake_minimum_required(VERSION 3.8)
project(savo_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

function(configure_savo_cpp_target target_name)
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${target_name}
    rclcpp
    std_msgs
    geometry_msgs
  )

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_executable(vl53_mux_node
  src/drivers/tca9548a.cpp
  src/drivers/vl53l1x_driver.cpp
  src/nodes/vl53_mux_node.cpp
)
configure_savo_cpp_target(vl53_mux_node)

option(SAVO_PERCEPTION_REQUIRE_VL53L1X_ULD
  "Require VL53L1X ULD files for real robot ToF C++ hardware"
  OFF
)

set(VL53L1X_ULD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l1x_uld")
set(VL53L1X_ULD_INCLUDE_DIR "${VL53L1X_ULD_DIR}/include")
set(VL53L1X_ULD_PLATFORM_DIR "${VL53L1X_ULD_DIR}/platform")

set(VL53L1X_ULD_SOURCES
  "${VL53L1X_ULD_DIR}/src/VL53L1X_api.c"
  "${VL53L1X_ULD_DIR}/src/VL53L1X_calibration.c"
  "${VL53L1X_ULD_PLATFORM_DIR}/VL53L1X_platform.c"
)

if(
  EXISTS "${VL53L1X_ULD_INCLUDE_DIR}/VL53L1X_api.h" AND
  EXISTS "${VL53L1X_ULD_INCLUDE_DIR}/VL53L1X_calibration.h" AND
  EXISTS "${VL53L1X_ULD_INCLUDE_DIR}/VL53L1X_types.h" AND
  EXISTS "${VL53L1X_ULD_DIR}/src/VL53L1X_api.c" AND
  EXISTS "${VL53L1X_ULD_DIR}/src/VL53L1X_calibration.c" AND
  EXISTS "${VL53L1X_ULD_PLATFORM_DIR}/VL53L1X_platform.h" AND
  EXISTS "${VL53L1X_ULD_PLATFORM_DIR}/VL53L1X_platform.c"
)
  target_sources(vl53_mux_node PRIVATE ${VL53L1X_ULD_SOURCES})
  target_include_directories(vl53_mux_node PRIVATE
    "${VL53L1X_ULD_INCLUDE_DIR}"
    "${VL53L1X_ULD_PLATFORM_DIR}"
  )
  target_compile_definitions(vl53_mux_node PRIVATE
    SAVO_PERCEPTION_HAVE_VL53L1X_ULD=1
  )
else()
  if(SAVO_PERCEPTION_REQUIRE_VL53L1X_ULD)
    message(FATAL_ERROR
      "VL53L1X ULD files are missing. Add third_party/vl53l1x_uld before real robot C++ ToF build.")
  else()
    message(WARNING
      "VL53L1X ULD files are missing; vl53_mux_node will build but real ToF hardware will not start.")
  endif()
endif()

add_executable(ultrasonic_node
  src/drivers/ultrasonic_reader.cpp
  src/nodes/ultrasonic_node.cpp
)
configure_savo_cpp_target(ultrasonic_node)

option(SAVO_PERCEPTION_REQUIRE_LGPIO
  "Require lgpio development files for real robot ultrasonic C++ hardware"
  OFF
)

find_path(LGPIO_INCLUDE_DIR lgpio.h)
find_library(LGPIO_LIBRARY lgpio)

if(LGPIO_INCLUDE_DIR AND LGPIO_LIBRARY)
  target_include_directories(ultrasonic_node PRIVATE ${LGPIO_INCLUDE_DIR})
  target_link_libraries(ultrasonic_node ${LGPIO_LIBRARY})
  target_compile_definitions(ultrasonic_node PRIVATE SAVO_PERCEPTION_HAVE_LGPIO=1)
else()
  if(SAVO_PERCEPTION_REQUIRE_LGPIO)
    message(FATAL_ERROR
      "lgpio development files were not found. Install lgpio headers/library before real robot build.")
  else()
    message(WARNING
      "lgpio development files were not found; ultrasonic_node will build without hardware support")
  endif()
endif()

add_executable(safety_stop_node
  src/fusion/range_fusion.cpp
  src/safety/safety_policy.cpp
  src/safety/safety_state.cpp
  src/nodes/safety_stop_node.cpp
)
configure_savo_cpp_target(safety_stop_node)

add_executable(cmd_vel_safety_gate
  src/safety/safety_gate.cpp
  src/nodes/cmd_vel_safety_gate.cpp
)
configure_savo_cpp_target(cmd_vel_safety_gate)

add_executable(range_health_node
  src/nodes/range_health_node.cpp
)
configure_savo_cpp_target(range_health_node)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

function(install_program_if_exists source_path installed_name)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source_path}")
    install(PROGRAMS
      ${source_path}
      DESTINATION lib/${PROJECT_NAME}
      RENAME ${installed_name}
    )
  endif()
endfunction()

install_program_if_exists(
  "savo_perception/nodes/vl53_mux_node_py.py"
  "vl53_mux_node_py"
)

install_program_if_exists(
  "savo_perception/nodes/ultrasonic_node_py.py"
  "ultrasonic_node_py"
)

install_program_if_exists(
  "savo_perception/nodes/safety_stop_node_py.py"
  "safety_stop_node_py"
)

install_program_if_exists(
  "savo_perception/nodes/range_health_node_py.py"
  "range_health_node_py"
)

install_program_if_exists(
  "savo_perception/nodes/sensor_dashboard_node.py"
  "sensor_dashboard_node"
)

install_program_if_exists(
  "scripts/range_echo_cli.py"
  "range_echo_cli"
)

install_program_if_exists(
  "scripts/safety_inject_cli.py"
  "safety_inject_cli"
)

install_program_if_exists(
  "scripts/perception_smoke_test_cli.py"
  "perception_smoke_test_cli"
)

install_program_if_exists(
  "scripts/dump_effective_params.py"
  "dump_effective_params"
)

install_program_if_exists(
  "scripts/tof_mux_scan_cli.py"
  "tof_mux_scan_cli"
)

install_program_if_exists(
  "scripts/ultrasonic_sample_cli.py"
  "ultrasonic_sample_cli"
)

foreach(dir config launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip noisy style-only linters for this hybrid production package.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_flake8 REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  ament_flake8(
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg"
  )
endif()

ament_package()
