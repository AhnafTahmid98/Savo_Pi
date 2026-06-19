cmake_minimum_required(VERSION 3.8)
project(savo_mapping)

# -----------------------------------------------------------------------------
# Compiler defaults
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# -----------------------------------------------------------------------------
# Python package
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/__init__.py")
  ament_python_install_package(${PROJECT_NAME})
endif()

# -----------------------------------------------------------------------------
# Build helpers
# -----------------------------------------------------------------------------
function(add_cpp_node_if_ready target_name)
  set(source_files ${ARGN})
  set(target_ready TRUE)

  foreach(src_file IN LISTS source_files)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${src_file}")
      set(target_ready FALSE)
    endif()
  endforeach()

  if(target_ready)
    add_executable(${target_name}
      ${source_files}
    )

    target_include_directories(${target_name}
      PRIVATE
        include
    )

    target_compile_features(${target_name}
      PRIVATE
        cxx_std_17
    )

    ament_target_dependencies(${target_name}
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
      nav_msgs
      diagnostic_msgs
      visualization_msgs
      tf2
      tf2_ros
      tf2_geometry_msgs
      nav2_msgs
      action_msgs
      lifecycle_msgs
    )

    install(TARGETS
      ${target_name}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endfunction()

function(install_program_if_exists program_path)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${program_path}")
    install(PROGRAMS
      ${program_path}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endfunction()

function(install_program_if_exists_renamed program_path output_name)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${program_path}")
    install(PROGRAMS
      ${program_path}
      DESTINATION lib/${PROJECT_NAME}
      RENAME ${output_name}
    )
  endif()
endfunction()

# -----------------------------------------------------------------------------
# C++ executables
# -----------------------------------------------------------------------------
add_cpp_node_if_ready(mapping_supervisor_node
  src/nodes/mapping_supervisor_node.cpp
  src/core/mapping_state.cpp
  src/core/mapping_status.cpp
  src/core/map_quality.cpp
  src/core/readiness_monitor.cpp
  src/core/tf_monitor.cpp
  src/core/pointcloud_health.cpp
  src/ros/topic_names.cpp
)

add_cpp_node_if_ready(mapping_mode_manager_node
  src/nodes/mapping_mode_manager_node.cpp
  src/core/mapping_mode.cpp
  src/core/mapping_state.cpp
  src/core/mapping_status.cpp
  src/ros/topic_names.cpp
)

add_cpp_node_if_ready(frontier_explorer_node
  src/nodes/frontier_explorer_node.cpp
  src/core/frontier_detector.cpp
  src/core/exploration_goal_selector.cpp
  src/core/exploration_blacklist.cpp
  src/core/mapping_state.cpp
  src/ros/topic_names.cpp
)

add_cpp_node_if_ready(pointcloud_monitor_node
  src/nodes/pointcloud_monitor_node.cpp
  src/core/pointcloud_health.cpp
  src/core/tf_monitor.cpp
  src/ros/topic_names.cpp
)

add_cpp_node_if_ready(semantic_landmark_recorder_node
  src/nodes/semantic_landmark_recorder_node.cpp
  src/core/semantic_landmark.cpp
  src/core/apriltag_observation.cpp
  src/core/location_record.cpp
  src/core/tf_monitor.cpp
  src/ros/topic_names.cpp
)

add_cpp_node_if_ready(location_bridge_node
  src/nodes/location_bridge_node.cpp
  src/core/semantic_landmark.cpp
  src/core/location_record.cpp
  src/ros/topic_names.cpp
)

# -----------------------------------------------------------------------------
# Python fallback nodes
# -----------------------------------------------------------------------------
install_program_if_exists_renamed(
  savo_mapping/nodes/mapping_supervisor_node.py
  mapping_supervisor_node_py
)

install_program_if_exists_renamed(
  savo_mapping/nodes/mapping_mode_manager_node.py
  mapping_mode_manager_node_py
)

install_program_if_exists_renamed(
  savo_mapping/nodes/frontier_explorer_node.py
  frontier_explorer_node_py
)

install_program_if_exists_renamed(
  savo_mapping/nodes/pointcloud_monitor_node.py
  pointcloud_monitor_node_py
)

install_program_if_exists_renamed(
  savo_mapping/nodes/semantic_landmark_recorder_node.py
  semantic_landmark_recorder_node_py
)

install_program_if_exists_renamed(
  savo_mapping/nodes/location_bridge_node.py
  location_bridge_node_py
)

# -----------------------------------------------------------------------------
# Python diagnostics and CLI
# -----------------------------------------------------------------------------
install_program_if_exists(savo_mapping/nodes/mapping_dashboard_node.py)
install_program_if_exists(savo_mapping/nodes/map_event_logger_node.py)

install_program_if_exists(scripts/dump_effective_params.py)
install_program_if_exists(scripts/mapping_smoke_test_cli.py)
install_program_if_exists(scripts/mapping_readiness_cli.py)
install_program_if_exists(scripts/manual_mapping_cli.py)
install_program_if_exists(scripts/autonomous_mapping_cli.py)

install_program_if_exists(scripts/save_map_cli.py)
install_program_if_exists(scripts/load_map_check_cli.py)
install_program_if_exists(scripts/map_quality_cli.py)
install_program_if_exists(scripts/map_metadata_cli.py)

install_program_if_exists(scripts/pointcloud_echo_cli.py)
install_program_if_exists(scripts/voxel_costmap_check_cli.py)

install_program_if_exists(scripts/apriltag_check_cli.py)
install_program_if_exists(scripts/semantic_landmark_cli.py)
install_program_if_exists(scripts/location_bridge_check_cli.py)
install_program_if_exists(scripts/clean_maps_cli.py)

# -----------------------------------------------------------------------------
# Install resources
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY
    include/
    DESTINATION include
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY
    rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY
    maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY
    resource/
    DESTINATION share/${PROJECT_NAME}/resource
  )
endif()

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

# -----------------------------------------------------------------------------
# Testing / Lint
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg")
    ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)
  endif()

  function(add_pytest_if_exists test_name test_path)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_path}")
      ament_add_pytest_test(${test_name} ${test_path})
    endif()
  endfunction()

  add_pytest_if_exists(test_imports test/test_imports.py)
  add_pytest_if_exists(test_topic_names test/test_topic_names.py)
  add_pytest_if_exists(test_topic_contract test/test_topic_contract.py)
  add_pytest_if_exists(test_param_loader test/test_param_loader.py)

  add_pytest_if_exists(test_mapping_models test/test_mapping_models.py)
  add_pytest_if_exists(test_mapping_mode test/test_mapping_mode.py)
  add_pytest_if_exists(test_readiness_state test/test_readiness_state.py)
  add_pytest_if_exists(test_map_quality test/test_map_quality.py)

  add_pytest_if_exists(test_frontier_detector test/test_frontier_detector.py)
  add_pytest_if_exists(test_goal_selector test/test_goal_selector.py)
  add_pytest_if_exists(test_exploration_blacklist test/test_exploration_blacklist.py)

  add_pytest_if_exists(test_pointcloud_status test/test_pointcloud_status.py)
  add_pytest_if_exists(test_apriltag_status test/test_apriltag_status.py)
  add_pytest_if_exists(test_semantic_landmark test/test_semantic_landmark.py)
  add_pytest_if_exists(test_location_record test/test_location_record.py)
endif()

# -----------------------------------------------------------------------------
# Package export
# -----------------------------------------------------------------------------
ament_package()