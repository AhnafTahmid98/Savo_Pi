cmake_minimum_required(VERSION 3.8)
project(savo_base)

# -----------------------------------------------------------------------------
# Compiler defaults (C++)
# -----------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # provides ament_python_install_package()

# Core ROS 2 (Python + optional C++)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(RapidJSON REQUIRED)

# Messages
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

# -----------------------------------------------------------------------------
# Python package install (modules/imports)
# -----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------------------------------------------------------
# C++ executables
# -----------------------------------------------------------------------------
add_executable(base_driver_node
  src/nodes/base_driver_node.cpp
  src/kinematics/mecanum_mixer.cpp
  src/drivers/freenove_motor_board.cpp
  src/safety/base_safety.cpp
  src/safety/command_guard.cpp
  src/safety/timeout_watchdog.cpp
  src/state/base_state.cpp
)

target_include_directories(base_driver_node
  PRIVATE
    include
)

target_compile_features(base_driver_node
  PRIVATE
    cxx_std_17
)

ament_target_dependencies(base_driver_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Aggregator core is independent of ROS so compatibility tests run on a PC.
add_library(base_state_summary STATIC src/state/state_summary.cpp)
target_include_directories(base_state_summary PUBLIC include)
target_include_directories(base_state_summary SYSTEM PRIVATE ${RAPIDJSON_INCLUDE_DIRS})
target_compile_features(base_state_summary PUBLIC cxx_std_17)

add_executable(base_state_publisher_node_cpp src/nodes/base_state_publisher_node.cpp)
target_link_libraries(base_state_publisher_node_cpp base_state_summary)
ament_target_dependencies(base_state_publisher_node_cpp rclcpp geometry_msgs std_msgs)

install(TARGETS
  base_driver_node
  base_state_publisher_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# -----------------------------------------------------------------------------
# Install Python executables (reliable for hybrid packages)
# -----------------------------------------------------------------------------
install(PROGRAMS
  # CLI tools (scripts/)
  scripts/teleop_letters_cli.py
  scripts/poke_test_cli.py
  scripts/mecanum_pattern_test_cli.py
  scripts/base_smoke_test_cli.py
  scripts/dump_effective_params.py

  # ROS nodes (savo_base/nodes/)
  savo_base/nodes/base_watchdog_node.py
  savo_base/nodes/base_state_publisher_node.py
  savo_base/nodes/base_diag_runner_node.py
  savo_base/nodes/base_heartbeat_node.py

  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  savo_base/nodes/base_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME base_driver_node_py
)

# -----------------------------------------------------------------------------
# Install non-code resources
# -----------------------------------------------------------------------------
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resource")
  install(DIRECTORY
    resource/
    DESTINATION share/${PROJECT_NAME}/resource
  )
endif()

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# README/package.xml in install tree is useful for deployed robot troubleshooting
install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)


# -----------------------------------------------------------------------------
# Testing / Lint
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_flake8(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg)

  ament_add_gtest(test_driver_runtime
    test/test_driver_runtime.cpp
    src/state/base_state.cpp
  )
  target_include_directories(test_driver_runtime PRIVATE include)
  target_compile_features(test_driver_runtime PRIVATE cxx_std_17)

  ament_add_gtest(test_freenove_motor_board
    test/test_freenove_motor_board.cpp
    src/drivers/freenove_motor_board.cpp
  )
  target_include_directories(test_freenove_motor_board
    PRIVATE
      include
  )
  target_compile_features(test_freenove_motor_board
    PRIVATE
      cxx_std_17
  )

  function(add_pytest_if_exists test_name test_path)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_path}")
      ament_add_pytest_test(${test_name} ${test_path})
    endif()
  endfunction()

  ament_add_pytest_test(test_state_publisher_compatibility
    test/test_state_publisher_compatibility.py
    APPEND_ENV RAPIDJSON_INCLUDE_DIR=${RAPIDJSON_INCLUDE_DIRS})
  add_pytest_if_exists(test_state_publisher_launch test/test_state_publisher_launch.py)
  # Core Pi: separate pytest processes pass, while a combined process can lose
  # GetParameters responses. The Fast DDS timing cause is not independently
  # root-caused; isolate tests without changing middleware or runtime assertions.
  # Use ament_add_test directly because ament_add_pytest_test validates its path
  # as a file/directory and cannot accept an exact pytest parametrized node ID.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(mode IN ITEMS normal disabled simulated board)
    foreach(impl IN ITEMS py cpp)
      if(impl STREQUAL "py")
        set(executable "base_state_publisher_node.py")
      else()
        set(executable "base_state_publisher_node_cpp")
      endif()
      set(test_name "test_state_publisher_ros_${mode}_${impl}")
      set(test_node_id
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_state_publisher_ros.py::test_installed_node_contract[${mode}-${executable}]")
      set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${test_name}.xunit.xml")
      ament_add_test(${test_name}
        COMMAND "${Python3_EXECUTABLE}" -u -m pytest "${test_node_id}"
          -o "cache_dir=${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_pytest/${test_name}/.cache"
          "--junit-xml=${result_file}"
          "--junit-prefix=${PROJECT_NAME}"
        OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_pytest/${test_name}.txt"
        RESULT_FILE "${result_file}"
      )
      # Preserve pytest's nonzero exit for missing/empty collection (no skip code).
      set_tests_properties(${test_name} PROPERTIES RUN_SERIAL TRUE LABELS "pytest")
    endforeach()
  endforeach()

  add_pytest_if_exists(test_mecanum_mix test/test_mecanum_mix.py)
  add_pytest_if_exists(test_wheel_command test/test_wheel_command.py)
  add_pytest_if_exists(test_clamp_and_scaling test/test_clamp_and_scaling.py)
  add_pytest_if_exists(test_timeout_watchdog_logic test/test_timeout_watchdog_logic.py)
  add_pytest_if_exists(test_topic_names test/test_topic_names.py)
  add_pytest_if_exists(test_param_loader test/test_param_loader.py)
  add_pytest_if_exists(test_driver_dryrun test/test_driver_dryrun.py)
endif()

# -----------------------------------------------------------------------------
# Package export
# -----------------------------------------------------------------------------
ament_package()
