cmake_minimum_required(VERSION 3.8)

project(savo_nav VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(savo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(
  savo_nav_core
  SHARED
  src/core/control_mode_client.cpp
  src/core/control_recovery_guard.cpp
  src/core/coverage_execution_model.cpp
  src/core/coverage_path_validator.cpp
  src/core/coverage_progress_tracker.cpp
  src/core/recovery_bridge.cpp
  src/core/goal_admission_policy.cpp
  src/core/goal_arbiter.cpp
  src/core/goal_context.cpp
  src/core/goal_gateway.cpp
  src/core/goal_validator.cpp
  src/core/localization_monitor.cpp
  src/core/location_navigation_contract.cpp
  src/core/map_context.cpp
  src/core/nav2_health_monitor.cpp
  src/core/navigation_readiness.cpp
  src/core/navigation_result.cpp
  src/core/navigation_state.cpp
  src/core/safety_state_monitor.cpp
  src/core/sensor_freshness_monitor.cpp
  src/core/types.cpp
)

target_include_directories(
  savo_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(
  savo_nav_core
  PUBLIC
    cxx_std_17
)

add_library(
  savo_nav_coverage_ros_adapter
  SHARED
  src/ros/coverage_action_adapter.cpp
  src/ros/coverage_admission_proxy.cpp
  src/ros/coverage_follow_path_gateway.cpp
)

target_include_directories(
  savo_nav_coverage_ros_adapter
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(
  savo_nav_coverage_ros_adapter
  savo_nav_core
)

ament_target_dependencies(
  savo_nav_coverage_ros_adapter
  action_msgs
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  savo_msgs
)

add_executable(
  navigate_to_location_node
  src/nodes/navigate_to_location_node.cpp
)

target_link_libraries(
  navigate_to_location_node
  savo_nav_core
)

ament_target_dependencies(
  navigate_to_location_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  savo_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(
  navigation_readiness_node
  src/nodes/navigation_readiness_node.cpp
)

target_link_libraries(
  navigation_readiness_node
  savo_nav_core
)

ament_target_dependencies(
  navigation_readiness_node
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_msgs
)

add_executable(
  control_recovery_guard_node
  src/nodes/control_recovery_guard_node.cpp
)

target_link_libraries(
  control_recovery_guard_node
  savo_nav_core
)

ament_target_dependencies(
  control_recovery_guard_node
  rclcpp
  std_msgs
)

add_executable(
  goal_admission_gate_node
  src/nodes/goal_admission_gate_node.cpp
)

target_link_libraries(
  goal_admission_gate_node
  savo_nav_core
  savo_nav_coverage_ros_adapter
)

ament_target_dependencies(
  goal_admission_gate_node
  action_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  savo_msgs
  std_msgs
)

add_executable(
  goal_gateway_node
  src/nodes/goal_gateway_node.cpp
)

target_link_libraries(
  goal_gateway_node
  savo_nav_core
  savo_nav_coverage_ros_adapter
)

ament_target_dependencies(
  goal_gateway_node
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  savo_msgs
  std_msgs
)

install(
  TARGETS
    savo_nav_core
    savo_nav_coverage_ros_adapter
  EXPORT export_savo_nav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    navigation_readiness_node
    navigate_to_location_node
    control_recovery_guard_node
    goal_admission_gate_node
    goal_gateway_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.before_*" EXCLUDE
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.before_*" EXCLUDE
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  PROGRAMS
    scripts/run_location_integration_smoke
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
    LICENSE
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_savo_nav_core
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)
ament_export_dependencies(
  builtin_interfaces
  nav_msgs
  savo_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )

  ament_add_gtest(
    test_navigation_readiness
    test/unit/test_navigation_readiness.cpp
  )

  if(TARGET test_navigation_readiness)
    target_link_libraries(
      test_navigation_readiness
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_static_contracts
    test/unit/test_goal_context.cpp
    test/unit/test_map_context.cpp
    test/unit/test_navigation_result.cpp
    test/unit/test_navigation_state.cpp
    test/unit/test_types.cpp
  )

  if(TARGET test_static_contracts)
    target_link_libraries(
      test_static_contracts
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_readiness_core
    test/unit/test_dependency_monitors.cpp
    test/unit/test_navigation_readiness_evaluator.cpp
    test/unit/test_sensor_freshness_monitor.cpp
  )

  if(TARGET test_readiness_core)
    target_link_libraries(
      test_readiness_core
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_coverage_path_validator
    test/unit/test_coverage_path_validator.cpp
  )

  if(TARGET test_coverage_path_validator)
    target_link_libraries(
      test_coverage_path_validator
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_coverage_progress_tracker
    test/unit/test_coverage_progress_tracker.cpp
  )

  if(TARGET test_coverage_progress_tracker)
    target_link_libraries(
      test_coverage_progress_tracker
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_coverage_execution_model
    test/unit/test_coverage_execution_model.cpp
  )

  if(TARGET test_coverage_execution_model)
    target_link_libraries(
      test_coverage_execution_model
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_coverage_action_adapter
    test/unit/test_coverage_action_adapter.cpp
  )

  if(TARGET test_coverage_action_adapter)
    target_link_libraries(
      test_coverage_action_adapter
      savo_nav_coverage_ros_adapter
    )

    ament_target_dependencies(
      test_coverage_action_adapter
      geometry_msgs
      savo_msgs
    )
  endif()

  ament_add_gtest(
    test_goal_validation_arbitration
    test/unit/test_goal_arbiter.cpp
    test/unit/test_goal_validator.cpp
  )

  if(TARGET test_goal_validation_arbitration)
    target_link_libraries(
      test_goal_validation_arbitration
      savo_nav_core
    )
  endif()

  ament_add_gtest(
    test_goal_gateway
    test/unit/test_goal_gateway.cpp
  )

  if(TARGET test_goal_gateway)
    target_link_libraries(
      test_goal_gateway
      savo_nav_core
    )
  endif()

  add_executable(
    fake_nav2_server_node
    test/fixtures/fake_nav2_server_node.cpp
  )

  ament_target_dependencies(
    fake_nav2_server_node
    nav2_msgs
    rclcpp
    rclcpp_action
  )

  install(
    TARGETS fake_nav2_server_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(
    fake_location_nav2_server_node
    test/fixtures/fake_location_nav2_server_node.cpp
  )

  ament_target_dependencies(
    fake_location_nav2_server_node
    geometry_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
  )

  install(
    TARGETS fake_location_nav2_server_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_add_gtest(
    test_control_recovery
    test/unit/test_control_recovery.cpp
  )

  if(TARGET test_control_recovery)
    target_link_libraries(
      test_control_recovery
      savo_nav_core
    )
  endif()

  ament_add_pytest_test(
    test_phase7_control_recovery_contracts
    test/contracts/test_phase7_control_recovery_contracts.py
    TIMEOUT 60
  )

  ament_add_gtest(
    test_goal_admission_policy
    test/unit/test_goal_admission_policy.cpp
  )

  if(TARGET test_goal_admission_policy)
    target_link_libraries(
      test_goal_admission_policy
      savo_nav_core
    )
  endif()

  ament_add_pytest_test(
    test_phase7b_goal_admission_contracts
    test/contracts/test_phase7b_goal_admission_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase4l_b3_coverage_contracts
    test/contracts/test_phase4l_b3_coverage_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase4l_b3d_runtime_contracts
    test/contracts/test_phase4l_b3d_runtime_contracts.py
    TIMEOUT 60
  )

  add_executable(
    fake_follow_path_server_node
    test/fixtures/fake_follow_path_server_node.cpp
  )

  ament_target_dependencies(
    fake_follow_path_server_node
    nav2_msgs
    rclcpp
    rclcpp_action
    std_msgs
  )

  install(
    TARGETS
      fake_follow_path_server_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_add_pytest_test(
    test_phase4l_b3e_integration_contracts
    test/contracts/test_phase4l_b3e_integration_contracts.py
    TIMEOUT 60
  )

  add_launch_test(
    test/launch/test_phase4l_b3e_coverage_runtime_launch.py
    TIMEOUT 90
  )

  add_executable(
    fake_guarded_gateway_server_node
    test/fixtures/fake_guarded_gateway_server_node.cpp
  )

  ament_target_dependencies(
    fake_guarded_gateway_server_node
    nav2_msgs
    rclcpp
    rclcpp_action
    std_msgs
  )

  target_link_libraries(
    fake_guarded_gateway_server_node
    savo_nav_core
  )

  install(
    TARGETS fake_guarded_gateway_server_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_add_pytest_test(
    test_location_navigation_runtime_contract
    test/contracts/test_location_navigation_runtime_contract.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_source_headers
    test/contracts/test_source_headers.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase0_contracts
    test/contracts/test_phase0_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase3_readiness_contracts
    test/contracts/test_phase3_readiness_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase4_goal_contracts
    test/contracts/test_phase4_goal_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase5_nav2_contracts
    test/contracts/test_phase5_nav2_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase6_gateway_contracts
    test/contracts/test_phase6_gateway_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_saved_map_launch
    test/launch/test_saved_map_launch.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase7c_fault_launch_contracts
    test/contracts/test_phase7c_fault_launch_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase8_costmap_sensor_contracts
    test/contracts/test_phase8_costmap_sensor_contracts.py
    TIMEOUT 60
  )

  ament_add_gtest(
    test_location_navigation_contract
    test/unit/test_location_navigation_contract.cpp
  )

  if(TARGET test_location_navigation_contract)
    target_link_libraries(
      test_location_navigation_contract
      savo_nav_core
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
