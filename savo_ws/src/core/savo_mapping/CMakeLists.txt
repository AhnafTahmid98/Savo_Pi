cmake_minimum_required(VERSION 3.8)
project(savo_mapping VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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
  src/quality/map_quality_placeholder.cpp
  src/slam/slam_contract.cpp
  src/slam/simulated_scan_model.cpp
  src/workflow/manual_workflow_state.cpp
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
    simulated_slam_input_node
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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

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

ament_package()
