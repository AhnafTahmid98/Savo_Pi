cmake_minimum_required(VERSION 3.8)
project(savo_mapping VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}_core
  src/core/version.cpp
  src/core/mapping_mode.cpp
  src/core/exploration_mode.cpp
  src/core/workflow_phase.cpp
  src/core/session_state.cpp
  src/core/mapping_types.cpp
  src/core/mapping_status.cpp
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
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(
  FILES README.md package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_version
    test/test_version.cpp
  )
  target_link_libraries(test_version
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_mode
    test/test_mapping_mode.cpp
  )
  target_link_libraries(test_mapping_mode
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_exploration_mode
    test/test_exploration_mode.cpp
  )
  target_link_libraries(test_exploration_mode
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_workflow_phase
    test/test_workflow_phase.cpp
  )
  target_link_libraries(test_workflow_phase
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_session_state
    test/test_session_state.cpp
  )
  target_link_libraries(test_session_state
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_types
    test/test_mapping_types.cpp
  )
  target_link_libraries(test_mapping_types
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_mapping_status
    test/test_mapping_status.cpp
  )
  target_link_libraries(test_mapping_status
    ${PROJECT_NAME}_core
  )
endif()

ament_export_targets(${PROJECT_NAME}_core_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
