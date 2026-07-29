#ifndef SAVO_LOCATIONS__TYPES_HPP_
#define SAVO_LOCATIONS__TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string_view>

namespace savo_locations
{

enum class CandidateState : std::uint8_t
{
  kUnknown = 0U,
  kPendingReview = 1U,
  kApproved = 2U,
  kRejected = 3U,
};

enum class LocationState : std::uint8_t
{
  kUnknown = 0U,
  kApproved = 1U,
  kRetired = 2U,
};

enum class SemanticType : std::uint8_t
{
  kUnknown = 0U,
  kRoom,
  kCorridor,
  kReception,
  kLaboratory,
  kElevator,
  kDoorway,
  kStairwell,
  kRestroom,
  kOffice,
  kClassroom,
  kServicePoint,
  kChargingStation,
  kOther,
};

[[nodiscard]]
std::string_view to_string(CandidateState state) noexcept;

[[nodiscard]]
std::string_view to_string(LocationState state) noexcept;

[[nodiscard]]
std::string_view to_string(SemanticType type) noexcept;

[[nodiscard]]
std::optional<SemanticType> semantic_type_from_string(
  std::string_view value) noexcept;

[[nodiscard]]
bool is_terminal(CandidateState state) noexcept;

[[nodiscard]]
bool is_navigable(
  LocationState state,
  bool enabled) noexcept;

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__TYPES_HPP_
