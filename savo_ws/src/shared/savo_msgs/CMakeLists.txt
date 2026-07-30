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
# Core dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
# Interface files
# ---------------------------------------------------------------------------
set(msg_files
  "msg/NavState.msg"
  "msg/RobotStatus.msg"
  "msg/IntentResult.msg"
  "msg/AprilTagObservation.msg"
  "msg/LocationRecord.msg"
  "msg/LocationCandidate.msg"
  "msg/LocationEvent.msg"
)

set(action_files
  "action/ConfirmAprilTag.action"
  "action/ExecuteCoveragePath.action"
  "action/RotateToHeading.action"
  "action/RegisterMappedLocation.action"
  "action/NavigateToLocation.action"
)
set(srv_files
  "srv/ResolveLocation.srv"
  "srv/GetLocation.srv"
  "srv/ListLocations.srv"
  "srv/RegisterLocationCandidate.srv"
  "srv/ApproveLocation.srv"
  "srv/SetLocationEnabled.srv"
  "srv/RecoverLocationStorage.srv"
  "srv/AuthorizeLocationOperation.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES ${ROSIDL_DEPENDENCIES}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_apriltag_interfaces
    test/test_apriltag_interfaces.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_rotate_to_heading_action_contract
    test/test_rotate_to_heading_action_contract.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_rotate_to_heading_registration
    test/test_rotate_to_heading_registration.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_execute_coverage_path_action_contract
    test/test_execute_coverage_path_action_contract.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_execute_coverage_path_registration
    test/test_execute_coverage_path_registration.py
    TIMEOUT 60
  )
ament_add_pytest_test(
    test_location_interfaces
    test/test_location_interfaces.py
    TIMEOUT 60
  )

  ament_add_pytest_test(
    test_location_integration_interfaces
    test/test_location_integration_interfaces.py
    TIMEOUT 60
  )
endif()

# ---------------------------------------------------------------------------
# Export and package

# ---------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)

ament_package()
