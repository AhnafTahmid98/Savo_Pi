cmake_minimum_required(VERSION 3.8)
project(savo_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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

savo_add_node(distance_approach_node
  src/nodes/distance_approach_node.cpp
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(PROGRAMS
    scripts/control_topic_check_cli.py
    scripts/mode_cmd_cli.py
    scripts/control_smoke_test_cli.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
    USE_SOURCE_PERMISSIONS
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  function(add_pytest_if_exists test_name test_file)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}")
      ament_add_pytest_test(${test_name}
        "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}"
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
        TIMEOUT 60
      )
    endif()
  endfunction()

  add_pytest_if_exists(test_angle_wrap test/test_angle_wrap.py)
  add_pytest_if_exists(test_command_limiter test/test_command_limiter.py)
  add_pytest_if_exists(test_pid_math test/test_pid_math.py)
  add_pytest_if_exists(test_package_identity test/test_package_identity.py)
  add_pytest_if_exists(test_package_metadata test/test_package_metadata.py)
  add_pytest_if_exists(test_control_modes_interface test/test_control_modes_interface.py)
  add_pytest_if_exists(test_distance_pid_controller test/test_distance_pid_controller.py)
  add_pytest_if_exists(test_heading_pid_controller test/test_heading_pid_controller.py)
  add_pytest_if_exists(test_pid_controller test/test_pid_controller.py)
  add_pytest_if_exists(test_control_status_node_helpers test/test_control_status_node_helpers.py)
  add_pytest_if_exists(test_recovery_status_node_compile test/test_recovery_status_node_compile.py)
  add_pytest_if_exists(test_control_dashboard_node_compile test/test_control_dashboard_node_compile.py)
  add_pytest_if_exists(test_distance_pid_test_node_compile test/test_distance_pid_test_node_compile.py)
  add_pytest_if_exists(test_keyboard_teleop_helpers test/test_keyboard_teleop_helpers.py)
  add_pytest_if_exists(test_keyboard_teleop_node_compile test/test_keyboard_teleop_node_compile.py)
  add_pytest_if_exists(test_auto_test_helpers test/test_auto_test_helpers.py)
  add_pytest_if_exists(test_auto_test_manager_node_compile test/test_auto_test_manager_node_compile.py)
  add_pytest_if_exists(test_recovery_test_helpers test/test_recovery_test_helpers.py)
  add_pytest_if_exists(test_recovery_test_manager_node_compile test/test_recovery_test_manager_node_compile.py)
  add_pytest_if_exists(test_straight_line_test_helpers test/test_straight_line_test_helpers.py)
  add_pytest_if_exists(test_straight_line_pid_test_node test/test_straight_line_pid_test_node.py)
  add_pytest_if_exists(test_launch_files test/test_launch_files.py)
  add_pytest_if_exists(test_config_files test/test_config_files.py)
  add_pytest_if_exists(test_control_mode_model test/test_control_mode_model.py)
  add_pytest_if_exists(test_topic_names test/test_topic_names.py)
  add_pytest_if_exists(test_diagnostics_helpers test/test_diagnostics_helpers.py)
  add_pytest_if_exists(test_time_utils test/test_time_utils.py)
  add_pytest_if_exists(test_scripts test/test_scripts.py)
  add_pytest_if_exists(test_twist_adapter test/test_twist_adapter.py)
  add_pytest_if_exists(test_twist_command_model test/test_twist_command_model.py)
  add_pytest_if_exists(test_command_source_model test/test_command_source_model.py)
  add_pytest_if_exists(test_distance_approach_model test/test_distance_approach_model.py)
  add_pytest_if_exists(test_param_utils test/test_param_utils.py)
  add_pytest_if_exists(test_math_utils test/test_math_utils.py)
  add_pytest_if_exists(test_validation_utils test/test_validation_utils.py)
  add_pytest_if_exists(test_control_status_model test/test_control_status_model.py)
  add_pytest_if_exists(test_recovery_state_model test/test_recovery_state_model.py)
  add_pytest_if_exists(test_stuck_state_model test/test_stuck_state_model.py)
  add_pytest_if_exists(test_qos_profiles test/test_qos_profiles.py)
  add_pytest_if_exists(test_message_utils test/test_message_utils.py)
  add_pytest_if_exists(test_status_adapter test/test_status_adapter.py)
  add_pytest_if_exists(test_config_adapter test/test_config_adapter.py)
  add_pytest_if_exists(test_health_helpers test/test_health_helpers.py)
  add_pytest_if_exists(test_topic_probe test/test_topic_probe.py)
  add_pytest_if_exists(test_file_utils test/test_file_utils.py)

  ament_cppcheck()
  ament_lint_cmake()
  ament_uncrustify()
  ament_xmllint()
endif()

ament_package()
