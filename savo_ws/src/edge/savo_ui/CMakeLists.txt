cmake_minimum_required(VERSION 3.8)
project(savo_ui)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -----------------------------------------------------------------------------
# C++ executables
# -----------------------------------------------------------------------------
add_executable(ui_node
  src/main.cpp
  src/app/ui_node.cpp
  src/render/image_asset.cpp
  src/render/canvas.cpp
  src/render/preview_writer.cpp
  src/platform/framebuffer_display.cpp
)

target_include_directories(ui_node
  PRIVATE
    include
)

target_compile_features(ui_node
  PRIVATE
    cxx_std_17
)

ament_target_dependencies(ui_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS
  ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets")
  install(DIRECTORY
    assets/
    DESTINATION share/${PROJECT_NAME}/assets
  )
endif()

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing / lint
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
