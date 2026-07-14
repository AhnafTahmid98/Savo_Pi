#ifndef SAVO_SPEECH__VERSION_HPP_
#define SAVO_SPEECH__VERSION_HPP_

#include <cstdint>
#include <string_view>

namespace savo_speech
{
namespace version
{

// -----------------------------------------------------------------------------
// Semantic package version
// Keep synchronized with package.xml and CMakeLists.txt.
// -----------------------------------------------------------------------------

inline constexpr std::uint32_t kMajor{0U};
inline constexpr std::uint32_t kMinor{1U};
inline constexpr std::uint32_t kPatch{0U};

inline constexpr std::string_view kPrerelease{""};
inline constexpr std::string_view kBuildMetadata{""};

inline constexpr std::string_view kVersion{"0.1.0"};

// -----------------------------------------------------------------------------
// Robot Savo package identity
// Matches savo_base and savo_control.
// -----------------------------------------------------------------------------

inline constexpr std::string_view kPackageName{"savo_speech"};
inline constexpr std::string_view kRobotName{"Robot Savo"};
inline constexpr std::string_view kOrganizationName{"Robot SAVO Project"};
inline constexpr std::string_view kSupportedRosDistro{"jazzy"};

struct PackageVersionInfo
{
  std::string_view package_name;
  std::string_view robot_name;
  std::string_view organization_name;
  std::string_view ros_distro;
  std::string_view version;

  std::uint32_t major;
  std::uint32_t minor;
  std::uint32_t patch;
};

[[nodiscard]] inline constexpr PackageVersionInfo package_version_info() noexcept
{
  return PackageVersionInfo{
    kPackageName,
    kRobotName,
    kOrganizationName,
    kSupportedRosDistro,
    kVersion,
    kMajor,
    kMinor,
    kPatch};
}

}  // namespace version
}  // namespace savo_speech

#endif  // SAVO_SPEECH__VERSION_HPP_
