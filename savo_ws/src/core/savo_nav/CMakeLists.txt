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

# Phase 0 contains package identity and frozen repository contracts only.
# Runtime libraries and executables will be added in later implementation phases.

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
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
  FILES
    README.md
    LICENSE
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)

  # ament_copyright does not recognize proprietary SPDX identifiers.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )

  if(EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/test/contracts/test_phase0_contracts.py")
    ament_add_pytest_test(
      test_phase0_contracts
      test/contracts/test_phase0_contracts.py
      TIMEOUT 60
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
