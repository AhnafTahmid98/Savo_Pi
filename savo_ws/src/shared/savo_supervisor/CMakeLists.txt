cmake_minimum_required(VERSION 3.8)
project(savo_supervisor VERSION 0.1.0 LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Compiler defaults (C++)
# -----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(savo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# Library targets
# -----------------------------------------------------------------------------
add_library(savo_supervisor_core
  src/freshness_tracker.cpp
  src/localization_payload_parser.cpp
  src/location_authorization_policy.cpp
  src/supervisor_policy.cpp
  src/transition_tracker.cpp
)

target_include_directories(savo_supervisor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(savo_supervisor_core
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(savo_supervisor_core
  rclcpp
  builtin_interfaces
)

target_link_libraries(savo_supervisor_core
  nlohmann_json::nlohmann_json
)

# -----------------------------------------------------------------------------
# Node target
# -----------------------------------------------------------------------------
add_executable(supervisor_node
  src/supervisor_node.cpp
)

target_compile_features(supervisor_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(supervisor_node
  savo_supervisor_core
)

ament_target_dependencies(supervisor_node
  rclcpp
  std_msgs
  diagnostic_msgs
  builtin_interfaces
  savo_msgs
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(TARGETS savo_supervisor_core
  EXPORT export_savo_supervisor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS supervisor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY resource/
    DESTINATION share/${PROJECT_NAME}/resource
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES README.md
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  # ament_copyright does not recognize LicenseRef-Proprietary.
  # Copyright notices remain enforced by cpplint.
  set(ament_cmake_copyright_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)

  # Cppcheck 2.13 cannot parse part of this GoogleTest source.
  # The test remains compiled and executed normally.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    "test/test_location_authorization_policy.cpp"
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_freshness_tracker
    test/test_freshness_tracker.cpp
  )
  target_link_libraries(test_freshness_tracker
    savo_supervisor_core
  )
  ament_target_dependencies(test_freshness_tracker
    rclcpp
    builtin_interfaces
  )

  ament_add_gtest(test_localization_payload_parser
    test/test_localization_payload_parser.cpp
  )
  target_link_libraries(test_localization_payload_parser
    savo_supervisor_core
  )
  ament_target_dependencies(test_localization_payload_parser
    builtin_interfaces
  )

  ament_add_gtest(test_transition_tracker
    test/test_transition_tracker.cpp
  )
  target_link_libraries(test_transition_tracker
    savo_supervisor_core
  )
  ament_target_dependencies(test_transition_tracker
    rclcpp
  )

  ament_add_gtest(test_supervisor_policy
    test/test_supervisor_policy.cpp
  )
  target_link_libraries(test_supervisor_policy
    savo_supervisor_core
  )
  ament_target_dependencies(test_supervisor_policy
    rclcpp
    builtin_interfaces
  )

  ament_add_gtest(test_supervisor_contract
    test/test_supervisor_contract.cpp
  )
  target_link_libraries(test_supervisor_contract
    savo_supervisor_core
  )
  ament_target_dependencies(test_supervisor_contract
    rclcpp
    builtin_interfaces
  )

  ament_add_gtest(test_location_authorization_policy
    test/test_location_authorization_policy.cpp
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_location_authorization_runtime_contract
    test/test_location_authorization_runtime_contract.py
    TIMEOUT 60
  )
  target_link_libraries(test_location_authorization_policy
    savo_supervisor_core
  )
endif()

ament_export_targets(
  export_savo_supervisor_core
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  builtin_interfaces
  nlohmann_json
  savo_msgs
)

ament_package()
