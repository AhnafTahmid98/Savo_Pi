cmake_minimum_required(VERSION 3.8)

project(savo_bridge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Werror=return-type
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/freshness_tracker.cpp
  src/graph_evidence.cpp
  src/health_evaluator.cpp
  src/observation_config.cpp
  src/snapshot_writer.cpp
  src/topic_observation.cpp
  src/version.cpp
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

add_library(${PROJECT_NAME}_ros STATIC
  src/bridge_node.cpp
  src/qos_profiles.cpp
  src/ros_graph_discovery.cpp
)

target_include_directories(${PROJECT_NAME}_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_ros
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}_ros
  diagnostic_msgs
  rclcpp
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_core
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_compile_features(${PROJECT_NAME}_node
  PRIVATE
    cxx_std_17
)

ament_target_dependencies(${PROJECT_NAME}_node
  diagnostic_msgs
  rclcpp
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_ros
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ros
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES
    README.md
    LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_savo_bridge_version
    test/unit/test_version.cpp
  )

  if(TARGET test_savo_bridge_version)
    target_link_libraries(test_savo_bridge_version
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_freshness
    test/unit/test_freshness_tracker.cpp
  )

  if(TARGET test_savo_bridge_freshness)
    target_link_libraries(test_savo_bridge_freshness
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_graph_evidence
    test/unit/test_graph_evidence.cpp
  )

  if(TARGET test_savo_bridge_graph_evidence)
    target_link_libraries(test_savo_bridge_graph_evidence
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_ros_graph_discovery
    test/unit/test_ros_graph_discovery.cpp
  )

  if(TARGET test_savo_bridge_ros_graph_discovery)
    target_link_libraries(test_savo_bridge_ros_graph_discovery
      ${PROJECT_NAME}_ros
      ${PROJECT_NAME}_core
    )

    ament_target_dependencies(
      test_savo_bridge_ros_graph_discovery
      rclcpp
      std_msgs
    )
  endif()

  ament_add_gtest(test_savo_bridge_snapshot_publication
    test/unit/test_bridge_snapshot_publication.cpp
  )

  if(TARGET test_savo_bridge_snapshot_publication)
    target_link_libraries(test_savo_bridge_snapshot_publication
      ${PROJECT_NAME}_ros
      ${PROJECT_NAME}_core
    )

    ament_target_dependencies(
      test_savo_bridge_snapshot_publication
      diagnostic_msgs
      rclcpp
      std_msgs
    )
  endif()

  ament_add_gtest(test_savo_bridge_generic_topic_observation
    test/unit/test_generic_topic_observation.cpp
  )

  if(TARGET test_savo_bridge_generic_topic_observation)
    target_link_libraries(
      test_savo_bridge_generic_topic_observation
      ${PROJECT_NAME}_ros
      ${PROJECT_NAME}_core
    )

    ament_target_dependencies(
      test_savo_bridge_generic_topic_observation
      diagnostic_msgs
      rclcpp
      std_msgs
    )
  endif()

  ament_add_gtest(test_savo_bridge_observation_config
    test/unit/test_observation_config.cpp
  )

  if(TARGET test_savo_bridge_observation_config)
    target_link_libraries(
      test_savo_bridge_observation_config
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_health_evaluator
    test/unit/test_health_evaluator.cpp
  )

  if(TARGET test_savo_bridge_health_evaluator)
    target_link_libraries(test_savo_bridge_health_evaluator
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_snapshot_writer
    test/unit/test_snapshot_writer.cpp
  )

  if(TARGET test_savo_bridge_snapshot_writer)
    target_link_libraries(test_savo_bridge_snapshot_writer
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_topic_observation
    test/unit/test_topic_observation.cpp
  )

  if(TARGET test_savo_bridge_topic_observation)
    target_link_libraries(test_savo_bridge_topic_observation
      ${PROJECT_NAME}_core
    )
  endif()

  ament_add_gtest(test_savo_bridge_qos
    test/unit/test_qos_profiles.cpp
  )

  if(TARGET test_savo_bridge_qos)
    ament_target_dependencies(test_savo_bridge_qos
      rclcpp
    )

    target_link_libraries(test_savo_bridge_qos
      ${PROJECT_NAME}_ros
    )
  endif()

  find_package(ament_lint_auto REQUIRED)

  # cppcheck 2.13.0 cannot parse the GTest TEST() macro in this
  # package even though GCC and GTest accept and execute the sources.
  # Register cppcheck explicitly so only production code is analyzed.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)

  # ament_copyright supports a fixed set of recognized open-source
  # license headers. Robot Savo is intentionally proprietary; the
  # package.xml license field and installed LICENSE file are the
  # authoritative license contract.
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cppcheck REQUIRED)

  # Restrict cppcheck to production source code. Ubuntu 24.04's
  # cppcheck 2.13.0 cannot parse GTest TEST() macros reliably, while
  # GCC, GTest, cpplint and uncrustify validate the test sources.
  ament_cppcheck(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    INCLUDE_DIRS
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # Ubuntu 24.04 provides cppcheck 2.13.0. The Jazzy wrapper marks this
  # version as slow by default, which produces skipped checks. Run it
  # explicitly so the package test result contains no skipped tests.
  if(TEST cppcheck)
    set_tests_properties(
      cppcheck
      PROPERTIES
        ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1"
    )
  endif()
endif()

ament_export_targets(
  ${PROJECT_NAME}_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  diagnostic_msgs
  rclcpp
  std_msgs
)
ament_export_include_directories(include)

ament_package()
