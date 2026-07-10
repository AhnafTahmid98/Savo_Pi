#pragma once

#include <string_view>
#include <tuple>

namespace savo_mapping::version
{

inline constexpr int VERSION_MAJOR = 0;
inline constexpr int VERSION_MINOR = 1;
inline constexpr int VERSION_PATCH = 0;

inline constexpr std::string_view PACKAGE_NAME = "savo_mapping";
inline constexpr std::string_view ROBOT_NAME = "Robot Savo";
inline constexpr std::string_view PROJECT_NAME = "Robot SAVO Project";
inline constexpr std::string_view SUPPORTED_ROS_DISTRO = "jazzy";
inline constexpr std::string_view VERSION = "0.1.0";

std::string_view get_package_name();
std::string_view get_robot_name();
std::string_view get_project_name();
std::string_view get_ros_distro();
std::string_view get_version();
std::tuple<int, int, int> get_version_tuple();

}  // namespace savo_mapping::version
