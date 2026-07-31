#ifndef SAVO_LOCATIONS__CONSTANTS_HPP_
#define SAVO_LOCATIONS__CONSTANTS_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace savo_locations
{

inline constexpr std::string_view kPackageName{
  "savo_locations"};

inline constexpr std::string_view kPackageVersion{
  "0.14.0"};

inline constexpr std::uint32_t kSchemaVersion{1U};

inline constexpr std::size_t
  kMaximumLocationIdLength{64U};

inline constexpr std::size_t
  kMaximumCandidateIdLength{128U};

inline constexpr std::size_t
  kMaximumDisplayNameLength{128U};

inline constexpr std::size_t
  kMaximumAliasCount{32U};

inline constexpr std::size_t
  kMaximumAliasLength{128U};

inline constexpr std::size_t
  kMaximumMapIdLength{128U};

inline constexpr std::size_t
  kMaximumMapReleaseIdLength{128U};

inline constexpr std::size_t
  kMaximumTagFamilyLength{64U};

inline constexpr std::string_view
  kCanonicalMapFrame{"map"};

inline constexpr double
  kQuaternionNormTolerance{0.01};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__CONSTANTS_HPP_
