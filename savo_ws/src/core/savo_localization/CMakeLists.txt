cmake_minimum_required(VERSION 3.8)
project(savo_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

find_library(LGPIO_LIBRARY lgpio)

set(IMU_NODE_SOURCES
  src/i2c_bus.cpp
  src/bno055_driver.cpp
  src/imu_node.cpp
)

set(IMU_NODE_READY TRUE)
foreach(src_file ${IMU_NODE_SOURCES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${src_file}")
    set(IMU_NODE_READY FALSE)
  endif()
endforeach()

if(IMU_NODE_READY)
  add_executable(imu_node ${IMU_NODE_SOURCES})

  ament_target_dependencies(imu_node
    rclcpp
    std_msgs
    sensor_msgs
    diagnostic_msgs
  )

  install(TARGETS
    imu_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

set(WHEEL_ODOM_SOURCES
  src/encoder_state.cpp
  src/encoder_reader.cpp
  src/mecanum_odom.cpp
  src/gpio_encoder_backend.cpp
  src/wheel_odom_node.cpp
)

set(WHEEL_ODOM_READY TRUE)
foreach(src_file ${WHEEL_ODOM_SOURCES})
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${src_file}")
    set(WHEEL_ODOM_READY FALSE)
  endif()
endforeach()

if(WHEEL_ODOM_READY)
  add_executable(wheel_odom_node ${WHEEL_ODOM_SOURCES})

  ament_target_dependencies(wheel_odom_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    diagnostic_msgs
    tf2
    tf2_ros
  )

  if(LGPIO_LIBRARY)
    target_link_libraries(wheel_odom_node ${LGPIO_LIBRARY})
    target_compile_definitions(wheel_odom_node PRIVATE SAVO_LOCALIZATION_LINKED_LGPIO=1)
  endif()

  install(TARGETS
    wheel_odom_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

function(install_program_if_exists program_path)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${program_path}")
    install(PROGRAMS
      ${program_path}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endfunction()

install_program_if_exists(savo_localization/nodes/localization_dashboard.py)
install_program_if_exists(savo_localization/nodes/localization_health_node.py)
install_program_if_exists(savo_localization/nodes/ekf_state_publisher_node.py)
install_program_if_exists(savo_localization/nodes/wheel_odom_fallback_node.py)

install_program_if_exists(scripts/imu_smoke_test_cli.py)
install_program_if_exists(scripts/encoder_smoke_test_cli.py)
install_program_if_exists(scripts/encoder_direction_test_cli.py)
install_program_if_exists(scripts/wheel_odom_echo_cli.py)
install_program_if_exists(scripts/ekf_echo_cli.py)
install_program_if_exists(scripts/tf_check_cli.py)
install_program_if_exists(scripts/dump_effective_params.py)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  function(add_pytest_if_exists test_name test_path)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_path}")
      ament_add_pytest_test(${test_name} ${test_path})
    endif()
  endfunction()

  add_pytest_if_exists(test_angle_math test/test_angle_math.py)
  add_pytest_if_exists(test_encoder_math test/test_encoder_math.py)
  add_pytest_if_exists(test_mecanum_kinematics test/test_mecanum_kinematics.py)
  add_pytest_if_exists(test_covariance test/test_covariance.py)
  add_pytest_if_exists(test_odom_math test/test_odom_math.py)
  add_pytest_if_exists(test_frames test/test_frames.py)
  add_pytest_if_exists(test_topic_contract test/test_topic_contract.py)
  add_pytest_if_exists(test_param_loader test/test_param_loader.py)
  add_pytest_if_exists(test_imu_models test/test_imu_models.py)
  add_pytest_if_exists(test_encoder_models test/test_encoder_models.py)
  add_pytest_if_exists(test_wheel_odom_models test/test_wheel_odom_models.py)
  add_pytest_if_exists(test_imu_health_check test/test_imu_health_check.py)
  add_pytest_if_exists(test_encoder_health_check test/test_encoder_health_check.py)
  add_pytest_if_exists(test_odom_sanity_check test/test_odom_sanity_check.py)
  add_pytest_if_exists(test_ekf_health_check test/test_ekf_health_check.py)
endif()

ament_package()