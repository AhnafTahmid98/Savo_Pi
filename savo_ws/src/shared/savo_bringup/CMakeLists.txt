cmake_minimum_required(VERSION 3.16)
project(savo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(SAVO_BRINGUP_DEPENDENCIES
  diagnostic_msgs
  rclcpp
  std_msgs
)

add_library(savo_bringup_contract SHARED
  src/bringup_contract.cpp
)

target_include_directories(savo_bringup_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(bringup_readiness_node
  src/nodes/bringup_readiness_node.cpp
)

target_include_directories(bringup_readiness_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(bringup_readiness_node
  savo_bringup_contract
)

ament_target_dependencies(bringup_readiness_node
  ${SAVO_BRINGUP_DEPENDENCIES}
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_location_lifecycle_runtime
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    savo_bringup_contract
    bringup_readiness_node
  EXPORT export_savo_bringup
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_savo_bringup HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # ament_copyright does not recognize LicenseRef-Proprietary; cpplint still
  # enforces the C++ copyright header while the package keeps its real license.
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  # Keep every substantive ament Flake8 rule while resolving the repository's
  # existing quote-style mismatch through the package-local Q000-only policy.
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")

  ament_add_gtest(test_bringup_contract
    test/test_bringup_contract.cpp
  )
  target_link_libraries(test_bringup_contract savo_bringup_contract)
  target_include_directories(test_bringup_contract PRIVATE include)

  # The location lifecycle runtime exercises Core-only packages such as
  # savo_locations, savo_mapping, savo_nav, and savo_supervisor. Keep shared
  # bringup tests role-neutral by registering that integration test only when
  # this package is configured as part of the Core role build.
  set(SAVO_BRINGUP_TEST_ROLE "$ENV{SAVO_ROLE}")
  if(SAVO_BRINGUP_TEST_ROLE STREQUAL "")
    set(SAVO_BRINGUP_TEST_ROLE "unspecified")
  endif()

  file(GLOB SAVO_BRINGUP_PYTEST_FILES test/test_*.py)
  foreach(test_file ${SAVO_BRINGUP_PYTEST_FILES})
    get_filename_component(test_name ${test_file} NAME_WE)

    if(
      "${test_name}" STREQUAL "test_location_lifecycle_runtime"
      AND NOT "${SAVO_BRINGUP_TEST_ROLE}" STREQUAL "core"
    )
      message(
        STATUS
        "Skipping Core-only ${test_name} for role=${SAVO_BRINGUP_TEST_ROLE}"
      )
    else()
      ament_add_pytest_test(${test_name} ${test_file}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
        TIMEOUT 120
      )
    endif()
  endforeach()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
