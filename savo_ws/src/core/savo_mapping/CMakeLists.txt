cmake_minimum_required(VERSION 3.8)
project(savo_mapping VERSION 0.1.0)

option(
  SAVO_MAPPING_BUILD_PC_SIMULATION
  "Build and install PC-only simulation tooling"
  OFF
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(savo_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/core/version.cpp
  src/core/mapping_mode.cpp
  src/core/exploration_mode.cpp
  src/core/exploration_goal_handoff.cpp
  src/core/workflow_phase.cpp
  src/core/session_state.cpp
  src/core/mapping_types.cpp
  src/core/mapping_status.cpp
  src/core/supervisor_runtime.cpp
  src/exploration/exploration_goal.cpp
  src/exploration/exploration_planner.cpp
  src/frontier/frontier_detector.cpp
  src/frontier/frontier_selector.cpp
  src/quality/map_quality_placeholder.cpp
  src/slam/slam_contract.cpp
  src/slam/simulated_scan_model.cpp
  src/workflow/manual_workflow_state.cpp
  src/workflow/workflow_authority.cpp
  src/workflow/exploration_runtime.cpp
  src/scan360/scan360_planner.cpp
  src/scan360/scan360_controller.cpp
  src/scan360/scan360_quality.cpp
  src/session/map_session_policy.cpp
  src/session/saved_map_contract.cpp
  src/session/saved_map_quality.cpp
  src/session/production_map_release.cpp
  src/ros/topic_names.cpp
  src/ros/qos_profiles.cpp
  src/ros/parameter_utils.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_core
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_core
  nav2_map_server
  nav_msgs
)

target_link_libraries(${PROJECT_NAME}_core yaml-cpp OpenSSL::Crypto)

add_executable(mapping_mode_manager_node
  src/nodes/mapping_mode_manager_node.cpp
)

target_compile_features(mapping_mode_manager_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(mapping_mode_manager_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(mapping_mode_manager_node
  rclcpp
  std_msgs
)

install(
  TARGETS mapping_mode_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(mapping_supervisor_node
  src/nodes/mapping_supervisor_node.cpp
)

target_compile_features(mapping_supervisor_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(mapping_supervisor_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(mapping_supervisor_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(
  TARGETS mapping_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(SAVO_MAPPING_BUILD_PC_SIMULATION)
  add_executable(simulated_slam_input_node
    src/nodes/simulated_slam_input_node.cpp
  )

  target_compile_features(simulated_slam_input_node
    PRIVATE
      cxx_std_17
  )

  target_link_libraries(simulated_slam_input_node
    ${PROJECT_NAME}_core
  )

  ament_target_dependencies(simulated_slam_input_node
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
  )

  install(
    TARGETS
      simulated_slam_input_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

add_executable(first_map_validator_node
  src/nodes/first_map_validator_node.cpp
)

target_compile_features(first_map_validator_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(first_map_validator_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(first_map_validator_node
  rclcpp
  std_msgs
  nav_msgs
)

install(
  TARGETS
    first_map_validator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

set(_savo_mapping_public_headers
  include/savo_mapping/scan360_rotate_action_client.hpp
  include/savo_mapping/exploration_goal_handoff.hpp
  include/savo_mapping/exploration_mode.hpp
  include/savo_mapping/exploration_goal.hpp
  include/savo_mapping/exploration_planner.hpp
  include/savo_mapping/frontier_detector.hpp
  include/savo_mapping/frontier_selector.hpp
  include/savo_mapping/manual_workflow_state.hpp
  include/savo_mapping/workflow_authority.hpp
  include/savo_mapping/exploration_runtime.hpp
  include/savo_mapping/scan360_planner.hpp
  include/savo_mapping/scan360_controller.hpp
  include/savo_mapping/scan360_quality.hpp
  include/savo_mapping/map_quality_placeholder.hpp
  include/savo_mapping/map_session_policy.hpp
  include/savo_mapping/mapping_mode.hpp
  include/savo_mapping/mapping_status.hpp
  include/savo_mapping/mapping_types.hpp
  include/savo_mapping/parameter_utils.hpp
  include/savo_mapping/production_map_release.hpp
  include/savo_mapping/qos_profiles.hpp
  include/savo_mapping/saved_map_contract.hpp
  include/savo_mapping/saved_map_quality.hpp
  include/savo_mapping/session_state.hpp
  include/savo_mapping/simulated_scan_model.hpp
  include/savo_mapping/slam_contract.hpp
  include/savo_mapping/slam_lifecycle_topics.hpp
  include/savo_mapping/supervisor_runtime.hpp
  include/savo_mapping/topic_names.hpp
  include/savo_mapping/version.hpp
  include/savo_mapping/workflow_phase.hpp
)

install(
  FILES ${_savo_mapping_public_headers}
  DESTINATION include/${PROJECT_NAME}
)

set(_savo_mapping_production_config
  config/exploration_goal_handoff.yaml
  config/exploration_manager.yaml
  config/frontier_mapping.yaml
  config/frames.yaml
  config/map_catalog_manager.yaml
  config/map_quality.yaml
  config/map_quality_evaluator.yaml
  config/map_session.yaml
  config/map_session_manager.yaml
  config/mapping_common.yaml
  config/mapping_mode_manager.yaml
  config/qos.yaml
  config/slam_lifecycle_health.yaml
  config/slam_toolbox_mapping.yaml
  config/topics.yaml
)

install(
  FILES ${_savo_mapping_production_config}
  DESTINATION share/${PROJECT_NAME}/config
)

set(_savo_mapping_production_profiles
  config/profiles/manual_mapping.yaml
  config/profiles/monitor_only.yaml
)

install(
  FILES ${_savo_mapping_production_profiles}
  DESTINATION share/${PROJECT_NAME}/config/profiles
)

set(_savo_mapping_production_launch
  launch/continued_mapping.launch.py
  launch/exploration_goal_handoff.launch.xml
  launch/frontier_mapping.launch.xml
  launch/manual_mapping.launch.xml
  launch/map_catalog_manager.launch.xml
  launch/map_quality_evaluator.launch.xml
  launch/map_session_manager.launch.xml
  launch/monitor_only.launch.xml
)

install(
  FILES ${_savo_mapping_production_launch}
  DESTINATION share/${PROJECT_NAME}/launch
)

if(SAVO_MAPPING_BUILD_PC_SIMULATION)
  set(_savo_mapping_simulation_profiles
    config/profiles/map_quality_simulated.yaml
    config/profiles/simulated_slam.yaml
  )

  install(
    FILES ${_savo_mapping_simulation_profiles}
    DESTINATION share/${PROJECT_NAME}/config/profiles
  )

  install(
    FILES launch/simulated_manual_mapping.launch.xml
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

install(
  FILES README.md package.xml
  DESTINATION share/${PROJECT_NAME}
)

add_executable(slam_lifecycle_health_bridge_node
  src/nodes/slam_lifecycle_health_bridge_node.cpp
)

target_compile_features(
  slam_lifecycle_health_bridge_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  slam_lifecycle_health_bridge_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  slam_lifecycle_health_bridge_node
  rclcpp
  std_msgs
  lifecycle_msgs
)

install(
  TARGETS
    slam_lifecycle_health_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_session_manager_node
  src/nodes/map_session_manager_node.cpp
)

target_compile_features(
  map_session_manager_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  map_session_manager_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  map_session_manager_node
  rclcpp
  std_msgs
  std_srvs
  slam_toolbox
)

install(
  TARGETS
    map_session_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(saved_map_verifier_node
  src/nodes/saved_map_verifier_node.cpp
)

target_compile_features(
  saved_map_verifier_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  saved_map_verifier_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  saved_map_verifier_node
  rclcpp
)

install(
  TARGETS
    saved_map_verifier_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_quality_evaluator_node
  src/nodes/map_quality_evaluator_node.cpp
)

target_compile_features(
  map_quality_evaluator_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  map_quality_evaluator_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  map_quality_evaluator_node
  rclcpp
  std_msgs
  std_srvs
)

install(
  TARGETS
    map_quality_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(map_catalog_manager_node
  src/nodes/map_catalog_manager_node.cpp
)

target_compile_features(
  map_catalog_manager_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  map_catalog_manager_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  map_catalog_manager_node
  rclcpp
  std_msgs
  std_srvs
)

install(
  TARGETS
    map_catalog_manager_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(exploration_manager_node
  src/nodes/exploration_manager_node.cpp
)

target_compile_features(
  exploration_manager_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  exploration_manager_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  exploration_manager_node
  rclcpp
  std_msgs
  std_srvs
)

install(
  TARGETS
    exploration_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(frontier_explorer_node
  src/nodes/frontier_explorer_node.cpp
)

target_compile_features(
  frontier_explorer_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  frontier_explorer_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  frontier_explorer_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    frontier_explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(exploration_goal_handoff_node
  src/nodes/exploration_goal_handoff_node.cpp
)

target_compile_features(
  exploration_goal_handoff_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  exploration_goal_handoff_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(
  exploration_goal_handoff_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
)

install(
  TARGETS
    exploration_goal_handoff_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(savo_mapping_scan360_rotate_action_client
  src/ros/scan360_rotate_action_client.cpp
)

target_include_directories(
  savo_mapping_scan360_rotate_action_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  savo_mapping_scan360_rotate_action_client
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  savo_mapping_scan360_rotate_action_client
  rclcpp
  rclcpp_action
  savo_msgs
)

install(
  TARGETS
    savo_mapping_scan360_rotate_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Zero-byte files are deferred phase scaffolds, not implemented code.
  # As soon as a scaffold gains content, it automatically enters linting.
  file(GLOB_RECURSE _savo_mapping_lint_candidates
    CONFIGURE_DEPENDS
    LIST_DIRECTORIES false
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/launch/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/rviz/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/*"
  )

  foreach(_lint_candidate IN LISTS _savo_mapping_lint_candidates)
    file(READ
      "${CMAKE_CURRENT_SOURCE_DIR}/${_lint_candidate}"
      _lint_first_byte
      LIMIT 1
    )

    if(_lint_first_byte STREQUAL "")
      list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
        "${CMAKE_CURRENT_SOURCE_DIR}/${_lint_candidate}"
      )
    elseif(_lint_candidate MATCHES
        "[.](c|cc|cpp|cxx|h|hh|hpp|hxx)$")
      list(APPEND _savo_mapping_active_cpp_files
        "${CMAKE_CURRENT_SOURCE_DIR}/${_lint_candidate}"
      )
    endif()
  endforeach()

  list(SORT AMENT_LINT_AUTO_FILE_EXCLUDE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_xmllint
  )

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(
    ${_savo_mapping_active_cpp_files}
    EXCLUDE
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_scan360_rotate_action_client_runtime.cpp"
    LANGUAGE c++
    INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  set_tests_properties(
    cppcheck
    PROPERTIES
      ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1"
  )

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    ${_savo_mapping_active_cpp_files}
    FILTERS
      "-build/include_order"
      "-legal/copyright"
  )

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(
    package.xml
    launch/exploration_goal_handoff.launch.xml
    launch/frontier_mapping.launch.xml
    launch/manual_mapping.launch.xml
    launch/map_catalog_manager.launch.xml
    launch/map_quality_evaluator.launch.xml
    launch/map_session_manager.launch.xml
    launch/monitor_only.launch.xml
    launch/simulated_manual_mapping.launch.xml
  )

  ament_add_pytest_test(
    test_scan360_rotate_action_client_contract
    test/test_scan360_rotate_action_client_contract.py
    TIMEOUT 60
  )

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_scan360_rotate_action_client_runtime
    test/test_scan360_rotate_action_client_runtime.cpp
  )

  if(TARGET test_scan360_rotate_action_client_runtime)
    target_link_libraries(
      test_scan360_rotate_action_client_runtime
      savo_mapping_scan360_rotate_action_client
    )

    ament_target_dependencies(
      test_scan360_rotate_action_client_runtime
      rclcpp
      rclcpp_action
      savo_msgs
    )

    set_tests_properties(
      test_scan360_rotate_action_client_runtime
      PROPERTIES
        TIMEOUT 60
    )
  endif()


  add_executable(
    fake_savo_nav_exploration_server_node
    test/nodes/fake_savo_nav_exploration_server_node.cpp
  )

  target_compile_features(
    fake_savo_nav_exploration_server_node
    PRIVATE
      cxx_std_17
  )

  ament_target_dependencies(
    fake_savo_nav_exploration_server_node
    nav2_msgs
    rclcpp
    rclcpp_action
  )


  ament_add_gtest(test_version
    test/test_version.cpp
  )
  target_link_libraries(test_version
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_mode
    test/test_mapping_mode.cpp
  )
  target_link_libraries(test_mapping_mode
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_mode
    test/test_exploration_mode.cpp
  )
  target_link_libraries(test_exploration_mode
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_frontier_detector
    test/test_frontier_detector.cpp
  )
  target_link_libraries(test_frontier_detector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_frontier_selector
    test/test_frontier_selector.cpp
  )
  target_link_libraries(test_frontier_selector
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_planner
    test/test_exploration_planner.cpp
  )
  target_link_libraries(test_exploration_planner
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_workflow_phase
    test/test_workflow_phase.cpp
  )
  target_link_libraries(test_workflow_phase
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_session_state
    test/test_session_state.cpp
  )
  target_link_libraries(test_session_state
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_types
    test/test_mapping_types.cpp
  )
  target_link_libraries(test_mapping_types
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_status
    test/test_mapping_status.cpp
  )
  target_link_libraries(test_mapping_status
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_topic_names
    test/test_topic_names.cpp
  )
  target_link_libraries(test_topic_names
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_qos_profiles
    test/test_qos_profiles.cpp
  )
  target_link_libraries(test_qos_profiles
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_parameter_utils
    test/test_parameter_utils.cpp
  )
  target_link_libraries(test_parameter_utils
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_parameter_utils
    rclcpp
  )
  ament_add_gtest(test_config_contracts
    test/test_config_and_launch_files.cpp
  )
  target_compile_definitions(test_config_contracts
    PRIVATE
      SAVO_MAPPING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_config_contracts
    ${PROJECT_NAME}_core
  )
  ament_target_dependencies(test_config_contracts
    rclcpp
  )
  ament_add_gtest(test_mapping_mode_manager_contract
    test/test_mapping_mode_manager_contract.cpp
  )

  target_compile_definitions(
    test_mapping_mode_manager_contract
    PRIVATE
      SAVO_MAPPING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(
    test_mapping_mode_manager_contract
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_supervisor_runtime
    test/test_mapping_supervisor_runtime.cpp
  )
  target_link_libraries(test_mapping_supervisor_runtime
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_slam_contract
    test/test_slam_contract.cpp
  )
  target_link_libraries(test_slam_contract
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_simulated_scan_model
    test/test_simulated_scan_model.cpp
  )
  target_link_libraries(test_simulated_scan_model
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_manual_workflow_state
    test/test_manual_workflow_state.cpp
  )

  target_link_libraries(
    test_manual_workflow_state
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_workflow_authority
    test/test_workflow_authority.cpp
  )

  target_link_libraries(
    test_workflow_authority
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_runtime
    test/test_exploration_runtime.cpp
  )

  target_link_libraries(
    test_exploration_runtime
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_scan360_planner
    test/test_scan360_planner.cpp
  )

  target_link_libraries(
    test_scan360_planner
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_scan360_controller
    test/test_scan360_controller.cpp
  )

  target_link_libraries(
    test_scan360_controller
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_scan360_quality
    test/test_scan360_quality.cpp
  )

  target_link_libraries(
    test_scan360_quality
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_map_session_policy
    test/test_map_session_policy.cpp
  )

  target_link_libraries(
    test_map_session_policy
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_saved_map_contract
    test/test_saved_map_contract.cpp
  )

  target_link_libraries(
    test_saved_map_contract
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_saved_map_quality
    test/test_saved_map_quality.cpp
  )

  target_link_libraries(
    test_saved_map_quality
    ${PROJECT_NAME}_core
  )
  ament_add_gtest(test_production_map_release
    test/test_production_map_release.cpp
  )

  target_link_libraries(
    test_production_map_release
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_manager_contract
    test/test_exploration_manager_contract.cpp
  )

  target_compile_definitions(
    test_exploration_manager_contract
    PRIVATE
      SAVO_MAPPING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(
    test_exploration_manager_contract
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_frontier_explorer_contract
    test/test_frontier_explorer_contract.cpp
  )

  target_compile_definitions(
    test_frontier_explorer_contract
    PRIVATE
      SAVO_MAPPING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(
    test_frontier_explorer_contract
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_goal_handoff
    test/test_exploration_goal_handoff.cpp
  )

  target_link_libraries(
    test_exploration_goal_handoff
    ${PROJECT_NAME}_core
  )

endif()

ament_export_targets(${PROJECT_NAME}_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

ament_export_dependencies(rclcpp rclcpp_action savo_msgs)

ament_package()
