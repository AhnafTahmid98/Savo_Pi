#ifndef SAVO_LOCATIONS__NORMALIZATION_HPP_
#define SAVO_LOCATIONS__NORMALIZATION_HPP_

#include <string>
#include <string_view>

namespace savo_locations
{

[[nodiscard]]
bool is_ascii_whitespace(char value) noexcept;

[[nodiscard]]
std::string trim_ascii(
  std::string_view value);

[[nodiscard]]
std::string collapse_ascii_whitespace(
  std::string_view value);

[[nodiscard]]
std::string normalize_lookup_key(
  std::string_view value);

[[nodiscard]]
std::string canonicalize_location_id(
  std::string_view value);

[[nodiscard]]
bool is_canonical_location_id(
  std::string_view value) noexcept;

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__NORMALIZATION_HPP_
