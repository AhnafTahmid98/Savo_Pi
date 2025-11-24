cmake_minimum_required(VERSION 3.8)
project(savo_msgs)

# ---------------------------------------------------------------------------
# Compiler settings
# ---------------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# If any of your .msg files use builtin_interfaces (time, duration) or
# std_msgs types, keep these in DEPENDENCIES below.
set(ROSIDL_DEPENDENCIES
  builtin_interfaces
  std_msgs
)

# ---------------------------------------------------------------------------
# Message files
# ---------------------------------------------------------------------------
set(msg_files
  "msg/NavState.msg"
  "msg/RobotStatus.msg"
  "msg/IntentResult.msg"
)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${ROSIDL_DEPENDENCIES}
)

# ---------------------------------------------------------------------------
# Export + package
# ---------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)

ament_package()
