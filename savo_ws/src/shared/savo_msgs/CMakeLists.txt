cmake_minimum_required(VERSION 3.8)
project(savo_msgs)

# ---------------------------------------------------------------------------
# Compiler settings (future-proof, even if we add C++ nodes later)
# ---------------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------------------------------------------------------------------
# Core dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message type dependencies:
# - builtin_interfaces: for Time/Duration in messages
# - std_msgs         : for basic types (if we embed them later)
# - geometry_msgs    : for Pose, Twist, etc. (used in NavState / RobotStatus)
# - nav_msgs         : for Odometry (used in NavState)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(ROSIDL_DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  nav_msgs
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
