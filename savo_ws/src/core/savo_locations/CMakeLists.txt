cmake_minimum_required(VERSION 3.16)
project(savo_locations VERSION 0.14.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(savo_msgs REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Deterministic domain library
# ---------------------------------------------------------------------------

add_library(
  savo_locations_contracts
  src/types.cpp
  src/normalization.cpp
  src/validation.cpp
  src/registry.cpp
  src/location_catalog.cpp
)

add_library(
  savo_locations::contracts
  ALIAS savo_locations_contracts
)

target_include_directories(
  savo_locations_contracts
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  savo_locations_contracts
  PUBLIC
    cxx_std_17
)

# ---------------------------------------------------------------------------
# SQLite storage library
# ---------------------------------------------------------------------------

add_library(
  savo_locations_storage
  src/sqlite_store.cpp
  src/sqlite_repository.cpp
)

add_library(
  savo_locations::storage
  ALIAS savo_locations_storage
)

target_include_directories(
  savo_locations_storage
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  savo_locations_storage
  PUBLIC
    cxx_std_17
)

target_link_libraries(
  savo_locations_storage
  PUBLIC
    savo_locations_contracts
    SQLite::SQLite3
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target
      savo_locations_contracts
      savo_locations_storage)
    target_compile_options(
      ${target}
      PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

install(
  TARGETS
    savo_locations_contracts
    savo_locations_storage
  EXPORT
    export_savo_locations
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)

install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)

install(
  FILES
    README.md
    LICENSE
  DESTINATION
    share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# LOC-3A production read-only ROS runtime
# ---------------------------------------------------------------------------

add_library(
  savo_locations_ros
  STATIC
    src/read_only_catalog_view.cpp
    src/ros_conversions.cpp
    src/location_registry_node.cpp
)

target_compile_features(
  savo_locations_ros
  PUBLIC
    cxx_std_17
)

target_include_directories(
  savo_locations_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  savo_locations_ros
  PUBLIC
    savo_locations_storage
)

ament_target_dependencies(
  savo_locations_ros
  PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    builtin_interfaces
    savo_msgs
)

add_executable(
  savo_locations_node
  src/location_registry_main.cpp
)

target_compile_features(
  savo_locations_node
  PRIVATE
    cxx_std_17
)

target_link_libraries(
  savo_locations_node
  savo_locations_ros
)

ament_target_dependencies(
  savo_locations_node
  rclcpp
)

install(
  TARGETS
    savo_locations_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    savo_locations_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(
    test_location_types
    test/unit/test_types.cpp
  )

  if(TARGET test_location_types)
    target_link_libraries(
      test_location_types
      savo_locations_contracts
    )
  endif()

  ament_add_gtest(
    test_location_normalization
    test/unit/test_normalization.cpp
  )

  if(TARGET test_location_normalization)
    target_link_libraries(
      test_location_normalization
      savo_locations_contracts
    )
  endif()

  ament_add_gtest(
    test_location_validation
    test/unit/test_validation.cpp
  )

  if(TARGET test_location_validation)
    target_link_libraries(
      test_location_validation
      savo_locations_contracts
    )
  endif()

  ament_add_gtest(
    test_location_registry
    test/unit/test_registry.cpp
  )

  if(TARGET test_location_registry)
    target_link_libraries(
      test_location_registry
      savo_locations_contracts
    )
  endif()

  ament_add_gtest(
    test_location_catalog
    test/unit/test_location_catalog.cpp
  )

  if(TARGET test_location_catalog)
    target_link_libraries(
      test_location_catalog
      savo_locations_contracts
    )
  endif()

  ament_add_gtest(
    test_sqlite_store
    test/storage/test_sqlite_store.cpp
  )

  if(TARGET test_sqlite_store)
    target_link_libraries(
      test_sqlite_store
      savo_locations_storage
      SQLite::SQLite3
    )

    target_compile_definitions(
      test_sqlite_store
      PRIVATE
        "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
    )
  endif()

  ament_add_gtest(
    test_sqlite_repository
    test/storage/test_sqlite_repository.cpp
  )

  if(TARGET test_sqlite_repository)
    target_link_libraries(
      test_sqlite_repository
      savo_locations_storage
      SQLite::SQLite3
    )

    target_compile_definitions(
      test_sqlite_repository
      PRIVATE
        "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
    )
  endif()

  ament_add_gtest(
    test_persistent_catalog
    test/storage/test_persistent_catalog.cpp
  )

  if(TARGET test_persistent_catalog)
    target_link_libraries(
      test_persistent_catalog
      savo_locations_storage
      SQLite::SQLite3
    )

    target_compile_definitions(
      test_persistent_catalog
      PRIVATE
        "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
    )
  endif()

  ament_add_gtest(
    test_persistent_mutation_commits
    test/storage/test_persistent_mutation_commits.cpp
  )

  if(TARGET test_persistent_mutation_commits)
    target_link_libraries(
      test_persistent_mutation_commits
      savo_locations_storage
      SQLite::SQLite3
    )

    target_compile_definitions(
      test_persistent_mutation_commits
      PRIVATE
        "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
    )
  endif()

  ament_add_pytest_test(
    test_phase0_contracts
    test/contracts/test_phase0_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase1a_contracts
    test/contracts/test_phase1a_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase1b_contracts
    test/contracts/test_phase1b_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase1c_contracts
    test/contracts/test_phase1c_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase2a_contracts
    test/contracts/test_phase2a_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase2b_contracts
    test/contracts/test_phase2b_contracts.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_phase2c_contracts
    test/contracts/test_phase2c_contracts.py
    TIMEOUT 60
  )
ament_add_gtest(
  test_read_only_catalog_view
  test/ros/test_read_only_catalog_view.cpp
)

if(TARGET test_read_only_catalog_view)
  target_link_libraries(
    test_read_only_catalog_view
    savo_locations_ros
  )
endif()

ament_add_gtest(
  test_registry_node
  test/ros/test_registry_node.cpp
)

if(TARGET test_registry_node)
  target_link_libraries(
    test_registry_node
    savo_locations_ros
    SQLite::SQLite3
  )

  ament_target_dependencies(
    test_registry_node
    rclcpp
    std_msgs
    savo_msgs
  )

  target_compile_definitions(
    test_registry_node
    PRIVATE
      "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
  )
endif()

ament_add_gtest(
  test_registry_write_node
  test/ros/test_registry_write_node.cpp
)

if(TARGET test_registry_write_node)
  target_link_libraries(
    test_registry_write_node
    savo_locations_ros
    SQLite::SQLite3
  )

  ament_target_dependencies(
    test_registry_write_node
    rclcpp
    std_msgs
    savo_msgs
  )

  target_compile_definitions(
    test_registry_write_node
    PRIVATE
      "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
  )
endif()

ament_add_pytest_test(
  test_phase3a_contracts
  test/contracts/test_phase3a_contracts.py
  TIMEOUT 60
)

ament_add_pytest_test(
  test_phase3b1_contracts
  test/contracts/test_phase3b1_contracts.py
  TIMEOUT 60
)

ament_add_pytest_test(
  test_phase3b2_contracts
  test/contracts/test_phase3b2_contracts.py
  TIMEOUT 60
)


ament_add_gtest(
  test_registry_hardening_node
  test/ros/test_registry_hardening_node.cpp
)

if(TARGET test_registry_hardening_node)
  target_link_libraries(
    test_registry_hardening_node
    savo_locations_ros
    SQLite::SQLite3
  )

  ament_target_dependencies(
    test_registry_hardening_node
    rclcpp
    std_msgs
    savo_msgs
  )

  target_compile_definitions(
    test_registry_hardening_node
    PRIVATE
      "SAVO_LOCATIONS_TEST_DB_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\""
  )
endif()

ament_add_pytest_test(
  test_phase3p_hardening_contracts
  test/contracts/test_phase3p_hardening_contracts.py
  TIMEOUT 60
)

endif()

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------

ament_export_targets(
  export_savo_locations
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)

ament_export_dependencies(
  savo_msgs
  SQLite3
)

ament_package()
