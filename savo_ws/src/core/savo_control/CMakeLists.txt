cmake_minimum_required(VERSION 3.8)
project(savo_control)

# Mid-level control only; motor hardware stays in savo_base.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Keep warnings strict before testing on the robot.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(savo_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Only include .cpp files that actually exist in this package.
add_library(${PROJECT_NAME}_core
  src/controllers/heading_controller.cpp
  src/controllers/pid.cpp

  src/filters/command_limiter.cpp

  src/recovery/recovery_manager.cpp
  src/recovery/stuck_detector.cpp

  src/utils/control_math.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  builtin_interfaces
  savo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

function(savo_add_node target source_file)
  add_executable(${target} ${source_file})

  target_link_libraries(${target}
    ${PROJECT_NAME}_core
  )

  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    builtin_interfaces
    savo_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

savo_add_node(twist_mux_node
  src/nodes/twist_mux_node.cpp
)

savo_add_node(cmd_vel_shaper_node
  src/nodes/cmd_vel_shaper_node.cpp
)

savo_add_node(control_mode_manager_node
  src/nodes/control_mode_manager_node.cpp
)

savo_add_node(heading_pid_node
  src/nodes/heading_pid_node.cpp
)

savo_add_node(rotate_to_heading_node
  src/nodes/rotate_to_heading_node.cpp
)

savo_add_node(velocity_test_pattern_node
  src/nodes/velocity_test_pattern_node.cpp
)

savo_add_node(recovery_manager_node
  src/nodes/recovery_manager_node.cpp
)

savo_add_node(backup_escape_node
  src/nodes/backup_escape_node.cpp
)

savo_add_node(stuck_detector_node
  src/nodes/stuck_detector_node.cpp
)

ament_python_install_package(${PROJECT_NAME})

# Launch files use these by executable name, so install them into lib/.
install(PROGRAMS
  savo_control/nodes/auto_test_manager_node.py
  savo_control/nodes/control_dashboard_node.py
  savo_control/nodes/control_status_node.py
  savo_control/nodes/keyboard_teleop_node.py
  savo_control/nodes/recovery_status_node.py
  savo_control/nodes/recovery_test_manager_node.py
  savo_control/nodes/straight_line_pid_test_node.py
  savo_control/nodes/distance_pid_test_node.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
    USE_SOURCE_PERMISSIONS
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_angle_wrap
    test/test_angle_wrap.py
  )

  ament_add_pytest_test(test_command_limiter
    test/test_command_limiter.py
  )

  ament_add_pytest_test(test_pid_math
    test/test_pid_math.py
  )
endif()

ament_package()