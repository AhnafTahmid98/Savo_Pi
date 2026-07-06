cmake_minimum_required(VERSION 3.16)
project(savo_power)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(savo_msgs REQUIRED)

include_directories(include)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/savo_power/__init__.py")
  ament_python_install_package(${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

foreach(dir config launch scripts)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  diagnostic_msgs
  savo_msgs
)

ament_package()
