cmake_minimum_required(VERSION 3.8)
project(savo_speech)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Build and ROS dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# -----------------------------------------------------------------------------
# Python support package
# -----------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------------------------------------------------------
# C++ dry-run production nodes
# -----------------------------------------------------------------------------
add_executable(respeaker_audio_node
  src/respeaker_audio_node.cpp
)

target_include_directories(respeaker_audio_node PRIVATE
  include
)

ament_target_dependencies(respeaker_audio_node
  rclcpp
  std_msgs
  std_srvs
  diagnostic_msgs
)

add_executable(speech_manager_node
  src/speech_manager_node.cpp
)

target_include_directories(speech_manager_node PRIVATE
  include
)

ament_target_dependencies(speech_manager_node
  rclcpp
  std_msgs
  std_srvs
  diagnostic_msgs
)

install(TARGETS
  respeaker_audio_node
  speech_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/speech_dryrun_check.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME speech_dryrun_check
)

# -----------------------------------------------------------------------------
# Public headers and resources
# -----------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  README.md
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing and lint
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  function(add_pytest_if_exists test_name test_file)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}")
      ament_add_pytest_test(${test_name}
        "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}"
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
        TIMEOUT 60
      )
    endif()
  endfunction()

  add_pytest_if_exists(
    test_package_identity
    test/test_package_identity.py
  )

  add_pytest_if_exists(
    test_package_metadata
    test/test_package_metadata.py
  )

  add_pytest_if_exists(
    test_speech_states
    test/test_speech_states.py
  )

  add_pytest_if_exists(
    test_topic_contract
    test/test_topic_contract.py
  )

  add_pytest_if_exists(
    test_qos_profiles
    test/test_qos_profiles.py
  )

  add_pytest_if_exists(
    test_contract_parity
    test/test_contract_parity.py
  )

  add_pytest_if_exists(
    test_topics_yaml
    test/test_topics_yaml.py
  )

  add_pytest_if_exists(
    test_config_contract
    test/test_config_contract.py
  )

  add_pytest_if_exists(
    test_config_profiles
    test/test_config_profiles.py
  )

  add_pytest_if_exists(
    test_dryrun_source_contract
    test/test_dryrun_source_contract.py
  )

  add_pytest_if_exists(
    test_phase0d_launch_contract
    test/test_phase0d_launch_contract.py
  )

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_speech_contract.cpp")
    ament_add_gtest(test_speech_contract
      test/test_speech_contract.cpp
    )

    if(TARGET test_speech_contract)
      target_include_directories(test_speech_contract PRIVATE
        include
      )
    endif()
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_qos_contract.cpp")
    ament_add_gtest(test_qos_contract
      test/test_qos_contract.cpp
    )

    if(TARGET test_qos_contract)
      target_include_directories(test_qos_contract PRIVATE
        include
      )

      ament_target_dependencies(test_qos_contract
        rclcpp
      )
    endif()
  endif()

  ament_cppcheck()
  ament_lint_cmake()
  ament_uncrustify()
  ament_xmllint()
endif()

ament_export_include_directories(include)

ament_package()
