cmake_minimum_required(VERSION 3.8)
project(savo_speech VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ALSA REQUIRED)

# ------------------------------------------------------------------------------
# Hardware-independent audio primitives
# ------------------------------------------------------------------------------

add_library(savo_speech_audio_core STATIC
  src/audio/audio_level_meter.cpp
  src/audio/audio_ring_buffer.cpp
  src/audio/bounded_audio_queue.cpp
  src/audio/channel_selector.cpp
  src/audio/capture_pipeline.cpp
  src/audio/capture_worker.cpp
  src/audio/microphone_gate.cpp
  src/audio/playback_controller.cpp
  src/audio/playback_worker.cpp
  src/audio/wav_reader.cpp
  src/audio/wav_writer.cpp
)

target_include_directories(savo_speech_audio_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# ALSA device discovery
# ------------------------------------------------------------------------------

add_library(savo_speech_audio_device STATIC
  src/drivers/alsa_capture_stream.cpp
  src/drivers/alsa_device_resolver.cpp
  src/drivers/alsa_playback_stream.cpp
)

target_include_directories(savo_speech_audio_device
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(savo_speech_audio_device
  PUBLIC
    ALSA::ALSA
)

# ------------------------------------------------------------------------------
# Production node
# ------------------------------------------------------------------------------

add_executable(savo_speech_node
  src/main.cpp
  src/speech_node.cpp
)

target_include_directories(savo_speech_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(savo_speech_node
  rclcpp
  std_msgs
  diagnostic_msgs
)

# ------------------------------------------------------------------------------
# Engineering tools
# ------------------------------------------------------------------------------

add_executable(savo_speech_audio_device_probe
  src/tools/audio_device_probe.cpp
)

target_include_directories(savo_speech_audio_device_probe
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(savo_speech_audio_device_probe
  PRIVATE
    savo_speech_audio_device
)

add_executable(savo_speech_capture_test
  src/tools/capture_test_cli.cpp
)

target_include_directories(savo_speech_capture_test
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(savo_speech_capture_test
  PRIVATE
    savo_speech_audio_core
    savo_speech_audio_device
)

add_executable(savo_speech_playback_test
  src/tools/playback_test_cli.cpp
)

target_include_directories(savo_speech_playback_test
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(savo_speech_playback_test
  PRIVATE
    savo_speech_audio_core
    savo_speech_audio_device
)

add_executable(savo_speech_playback_worker_test
  src/tools/playback_worker_test_cli.cpp
)

target_include_directories(savo_speech_playback_worker_test
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(savo_speech_playback_worker_test
  PRIVATE
    savo_speech_audio_core
    savo_speech_audio_device
)

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------

install(
  TARGETS
    savo_speech_node
    savo_speech_audio_device_probe
    savo_speech_capture_test
    savo_speech_playback_test
    savo_speech_playback_worker_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    README.md
    CHANGELOG.md
    LICENSE
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Tests and lint
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_audio_primitives
    test/unit/test_audio_format.cpp
    test/unit/test_alsa_capture_config.cpp
    test/unit/test_alsa_playback_config.cpp
    test/unit/test_audio_buffer.cpp
    test/unit/test_audio_level_meter.cpp
    test/unit/test_microphone_gate.cpp
    test/unit/test_playback_controller.cpp
    test/unit/test_playback_worker.cpp
    test/unit/test_audio_ring_buffer.cpp
    test/unit/test_bounded_audio_queue.cpp
    test/unit/test_capture_pipeline.cpp
    test/unit/test_capture_worker.cpp
    test/unit/test_channel_selector.cpp
    test/unit/test_wav_codec.cpp
  )

  target_include_directories(test_audio_primitives
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(test_audio_primitives
    savo_speech_audio_core
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
