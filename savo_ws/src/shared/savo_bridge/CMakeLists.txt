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

  find_package(ament_lint_auto REQUIRED)

  # ament_copyright supports a fixed set of recognized open-source
  # license headers. Robot Savo is intentionally proprietary; the
  # package.xml license field and installed LICENSE file are the
  # authoritative license contract.
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

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
  ${PROJECT_NAME}_core_targets
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_package()
