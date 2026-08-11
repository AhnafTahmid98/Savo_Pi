cmake_minimum_required(VERSION 3.8)
project(savo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    config
    rviz
    meshes
    scripts
    docs
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES
    README.md
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_urdf_exists test/test_urdf_exists.py)
  ament_add_pytest_test(test_xacro_builds test/test_xacro_builds.py)
  ament_add_pytest_test(test_required_frames test/test_required_frames.py)
  ament_add_pytest_test(test_config_files test/test_config_files.py)
  ament_add_pytest_test(test_no_duplicate_frames test/test_no_duplicate_frames.py)
  ament_add_pytest_test(test_install_contract test/test_install_contract.py)
  ament_add_pytest_test(test_geometry_contract test/test_geometry_contract.py)
  ament_add_pytest_test(test_am0b_geometry_profile test/test_am0b_geometry_profile.py)
  ament_add_pytest_test(
    test_robot_savo_geometry_integration
    test/test_robot_savo_geometry_integration.py
  )
endif()

ament_package()
