#ifndef SAVO_LOCATIONS__VALIDATION_HPP_
#define SAVO_LOCATIONS__VALIDATION_HPP_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "savo_locations/model.hpp"

namespace savo_locations
{

enum class ValidationCode : std::uint8_t
{
  kNone = 0U,
  kEmptyValue,
  kTooLong,
  kInvalidFormat,
  kUnsupportedValue,
  kDuplicateNormalizedKey,
  kNonFiniteNumber,
  kOutOfRange,
  kMapRevisionZero,
  kWrongFrame,
  kInvalidQuaternion,
  kMissingRequiredPose,
};


struct ValidationIssue
{
  ValidationCode code{ValidationCode::kNone};
  std::string field;
  std::string message;
};


class ValidationResult
{
public:
  [[nodiscard]]
  bool valid() const noexcept;

  [[nodiscard]]
  bool has(
    ValidationCode code) const noexcept;

  [[nodiscard]]
  const std::vector<ValidationIssue> &
  issues() const noexcept;

  void add(
    ValidationCode code,
    std::string field,
    std::string message);

private:
  std::vector<ValidationIssue> issues_;
};


[[nodiscard]]
std::string_view to_string(
  ValidationCode code) noexcept;

[[nodiscard]]
ValidationResult validate_map_context(
  const MapContext & context);

[[nodiscard]]
ValidationResult validate_pose(
  const PoseData & pose,
  std::string_view field);

[[nodiscard]]
ValidationResult validate_tag_binding(
  const TagBinding & tag);

[[nodiscard]]
ValidationResult validate_location_draft(
  const LocationDraft & location);

[[nodiscard]]
ValidationResult validate_candidate_draft(
  const CandidateDraft & candidate);

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__VALIDATION_HPP_
