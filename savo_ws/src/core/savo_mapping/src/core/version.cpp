#include "savo_mapping/version.hpp"

namespace savo_mapping::version
{

std::string_view get_package_name()
{
  return PACKAGE_NAME;
}

std::string_view get_robot_name()
{
  return ROBOT_NAME;
}

std::string_view get_project_name()
{
  return PROJECT_NAME;
}

std::string_view get_ros_distro()
{
  return SUPPORTED_ROS_DISTRO;
}

std::string_view get_version()
{
  return VERSION;
}

std::tuple<int, int, int> get_version_tuple()
{
  return {VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH};
}

}  // namespace savo_mapping::version
