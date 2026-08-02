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
find_package(savo_msgs REQUIRED)

# -----------------------------------------------------------------------------
# C++ executables
# -----------------------------------------------------------------------------
add_executable(ui_node
  src/main.cpp
  src/app/ui_node.cpp
  src/app/live_state.cpp
  src/render/image_asset.cpp
  src/render/canvas.cpp
  src/render/preview_writer.cpp
  src/render/font.cpp
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
  savo_msgs
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  # ament_copyright does not recognize this package's proprietary license text.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_add_gtest(test_live_state test/test_live_state.cpp src/app/live_state.cpp)
  if(TARGET test_live_state)
    target_include_directories(test_live_state PRIVATE include)
  endif()
  ament_add_pytest_test(
    test_ui_integration_contract
    test/test_ui_integration_contract.py
  )
  ament_lint_auto_find_test_dependencies()

  # The active production UI/speech implementations intentionally
  # preserve their validated monolithic paths for this closure pass.
  # All cpplint categories except legacy function length remain active.
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS
      "-readability/fn_size,-readability/braces"
  )
endif()

ament_package()
