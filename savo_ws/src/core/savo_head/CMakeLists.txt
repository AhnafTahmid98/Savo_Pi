cmake_minimum_required(VERSION 3.8)
project(savo_head)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# -----------------------------------------------------------------------------
# Python package install
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/__init__.py")
  ament_python_install_package(${PROJECT_NAME})
endif()

# -----------------------------------------------------------------------------
# C++ executables
#
# Nodes are added only when their source files exist. This keeps the package
# buildable while we create the production nodes one file at a time.
# -----------------------------------------------------------------------------
function(add_savo_head_node target_name source_path)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${source_path}")
    add_executable(${target_name}
      ${source_path}
    )

    target_include_directories(${target_name}
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_compile_features(${target_name}
      PRIVATE
        cxx_std_17
    )

    ament_target_dependencies(${target_name}
      rclcpp
      std_msgs
      geometry_msgs
      sensor_msgs
      diagnostic_msgs
      std_srvs
      tf2_ros
    )

    install(TARGETS
      ${target_name}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endfunction()

add_savo_head_node(head_controller_node src/nodes/head_controller_node.cpp)
add_savo_head_node(head_scan_node src/nodes/head_scan_node.cpp)
add_savo_head_node(head_tf_node src/nodes/head_tf_node.cpp)
add_savo_head_node(head_status_node src/nodes/head_status_node.cpp)
add_savo_head_node(apriltag_confirm_node src/nodes/apriltag_confirm_node.cpp)

# -----------------------------------------------------------------------------
# Python fallback nodes and diagnostic tools
# -----------------------------------------------------------------------------
file(GLOB SCRIPTS_TO_INSTALL
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "scripts/*"
)

if(SCRIPTS_TO_INSTALL)
  install(PROGRAMS
    ${SCRIPTS_TO_INSTALL}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# -----------------------------------------------------------------------------
# Install headers and resources
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
    PATTERN "__pycache__" EXCLUDE
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
    PATTERN "__pycache__" EXCLUDE
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
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  file(GLOB PYTEST_FILES
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "test/test_*.py"
  )

  foreach(test_path ${PYTEST_FILES})
    get_filename_component(test_name "${test_path}" NAME_WE)
    ament_add_pytest_test(${test_name} "${test_path}")
  endforeach()
endif()

# -----------------------------------------------------------------------------
# Package export
# -----------------------------------------------------------------------------
ament_export_include_directories(include)

ament_package()
