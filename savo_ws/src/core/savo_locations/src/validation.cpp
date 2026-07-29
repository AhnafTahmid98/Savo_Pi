#include "savo_locations/validation.hpp"

#include <cmath>
#include <set>
#include <utility>

#include "savo_locations/constants.hpp"
#include "savo_locations/normalization.hpp"
#include "savo_locations/types.hpp"

namespace savo_locations
{
namespace
{

void append(
  ValidationResult & destination,
  const ValidationResult & source)
{
  for (const auto & issue : source.issues()) {
    destination.add(
      issue.code,
      issue.field,
      issue.message);
  }
}


void validate_required_text(
  ValidationResult & result,
  const std::string_view value,
  const std::string_view field,
  const std::size_t maximum_length)
{
  const auto trimmed = trim_ascii(value);

  if (trimmed.empty()) {
    result.add(
      ValidationCode::kEmptyValue,
      std::string{field},
      "value is required");

    return;
  }

  if (trimmed.size() > maximum_length) {
    result.add(
      ValidationCode::kTooLong,
      std::string{field},
      "value exceeds maximum length");
  }
}


void validate_optional_text(
  ValidationResult & result,
  const std::string_view value,
  const std::string_view field,
  const std::size_t maximum_length)
{
  if (value.empty()) {
    return;
  }

  if (
    trim_ascii(value).size() >
    maximum_length)
  {
    result.add(
      ValidationCode::kTooLong,
      std::string{field},
      "value exceeds maximum length");
  }
}


bool finite(
  const double value) noexcept
{
  return std::isfinite(value);
}


void validate_non_negative_finite(
  ValidationResult & result,
  const double value,
  const std::string_view field)
{
  if (!finite(value)) {
    result.add(
      ValidationCode::kNonFiniteNumber,
      std::string{field},
      "value must be finite");

    return;
  }

  if (value < 0.0) {
    result.add(
      ValidationCode::kOutOfRange,
      std::string{field},
      "value must be non-negative");
  }
}


void validate_identity_aliases(
  ValidationResult & result,
  const std::string_view location_id,
  const std::string_view display_name,
  const std::vector<std::string> & aliases,
  const std::string_view field_prefix)
{
  if (aliases.size() > kMaximumAliasCount) {
    result.add(
      ValidationCode::kTooLong,
      std::string{field_prefix},
      "alias count exceeds maximum");

    return;
  }

  std::set<std::string> normalized_keys;

  const auto normalized_id =
    normalize_lookup_key(location_id);

  const auto normalized_display_name =
    normalize_lookup_key(display_name);

  if (!normalized_id.empty()) {
    normalized_keys.insert(normalized_id);
  }

  if (!normalized_display_name.empty()) {
    normalized_keys.insert(
      normalized_display_name);
  }

  for (
    std::size_t index = 0U;
    index < aliases.size();
    ++index)
  {
    const auto field =
      std::string{field_prefix} +
      "[" +
      std::to_string(index) +
      "]";

    const auto trimmed =
      trim_ascii(aliases[index]);

    if (trimmed.empty()) {
      result.add(
        ValidationCode::kEmptyValue,
        field,
        "alias must not be empty");

      continue;
    }

    if (
      trimmed.size() >
      kMaximumAliasLength)
    {
      result.add(
        ValidationCode::kTooLong,
        field,
        "alias exceeds maximum length");

      continue;
    }

    const auto key =
      normalize_lookup_key(trimmed);

    if (key.empty()) {
      result.add(
        ValidationCode::kInvalidFormat,
        field,
        "alias does not contain a searchable key");

      continue;
    }

    const auto inserted =
      normalized_keys.insert(key);

    if (!inserted.second) {
      result.add(
        ValidationCode::
          kDuplicateNormalizedKey,
        field,
        "alias duplicates another identity key");
    }
  }
}


void validate_semantic_type(
  ValidationResult & result,
  const std::string_view value,
  const std::string_view field,
  const bool required)
{
  const auto trimmed = trim_ascii(value);

  if (trimmed.empty()) {
    if (required) {
      result.add(
        ValidationCode::kEmptyValue,
        std::string{field},
        "semantic type is required");
    }

    return;
  }

  const auto parsed =
    semantic_type_from_string(trimmed);

  if (!parsed.has_value()) {
    result.add(
      ValidationCode::kUnsupportedValue,
      std::string{field},
      "semantic type is not supported");
  }
}

}  // namespace


bool ValidationResult::valid() const noexcept
{
  return issues_.empty();
}


bool ValidationResult::has(
  const ValidationCode code) const noexcept
{
  for (const auto & issue : issues_) {
    if (issue.code == code) {
      return true;
    }
  }

  return false;
}


const std::vector<ValidationIssue> &
ValidationResult::issues() const noexcept
{
  return issues_;
}


void ValidationResult::add(
  const ValidationCode code,
  std::string field,
  std::string message)
{
  issues_.push_back(
    ValidationIssue{
      code,
      std::move(field),
      std::move(message)});
}


std::string_view to_string(
  const ValidationCode code) noexcept
{
  switch (code) {
    case ValidationCode::kNone:
      return "none";

    case ValidationCode::kEmptyValue:
      return "empty_value";

    case ValidationCode::kTooLong:
      return "too_long";

    case ValidationCode::kInvalidFormat:
      return "invalid_format";

    case ValidationCode::kUnsupportedValue:
      return "unsupported_value";

    case ValidationCode::
      kDuplicateNormalizedKey:
      return "duplicate_normalized_key";

    case ValidationCode::kNonFiniteNumber:
      return "non_finite_number";

    case ValidationCode::kOutOfRange:
      return "out_of_range";

    case ValidationCode::kMapRevisionZero:
      return "map_revision_zero";

    case ValidationCode::kWrongFrame:
      return "wrong_frame";

    case ValidationCode::kInvalidQuaternion:
      return "invalid_quaternion";

    case ValidationCode::kMissingRequiredPose:
      return "missing_required_pose";

    default:
      return "unknown";
  }
}


ValidationResult validate_map_context(
  const MapContext & context)
{
  ValidationResult result;

  validate_required_text(
    result,
    context.map_id,
    "map.map_id",
    kMaximumMapIdLength);

  if (context.map_revision == 0U) {
    result.add(
      ValidationCode::kMapRevisionZero,
      "map.map_revision",
      "map revision must be greater than zero");
  }

  validate_optional_text(
    result,
    context.map_release_id,
    "map.map_release_id",
    kMaximumMapReleaseIdLength);

  return result;
}


ValidationResult validate_pose(
  const PoseData & pose,
  const std::string_view field)
{
  ValidationResult result;

  if (pose.frame_id != kCanonicalMapFrame) {
    result.add(
      ValidationCode::kWrongFrame,
      std::string{field} + ".frame_id",
      "pose frame must be map");
  }

  const bool all_finite =
    finite(pose.x) &&
    finite(pose.y) &&
    finite(pose.z) &&
    finite(pose.qx) &&
    finite(pose.qy) &&
    finite(pose.qz) &&
    finite(pose.qw);

  if (!all_finite) {
    result.add(
      ValidationCode::kNonFiniteNumber,
      std::string{field},
      "pose values must all be finite");

    return result;
  }

  const double norm_squared =
    pose.qx * pose.qx +
    pose.qy * pose.qy +
    pose.qz * pose.qz +
    pose.qw * pose.qw;

  const double norm =
    std::sqrt(norm_squared);

  if (
    std::abs(norm - 1.0) >
    kQuaternionNormTolerance)
  {
    result.add(
      ValidationCode::kInvalidQuaternion,
      std::string{field} + ".orientation",
      "quaternion must be normalized");
  }

  return result;
}


ValidationResult validate_tag_binding(
  const TagBinding & tag)
{
  ValidationResult result;

  validate_required_text(
    result,
    tag.family,
    "tag.family",
    kMaximumTagFamilyLength);

  if (tag.id < 0) {
    result.add(
      ValidationCode::kOutOfRange,
      "tag.id",
      "AprilTag ID must be non-negative");
  }

  return result;
}


ValidationResult validate_location_draft(
  const LocationDraft & location)
{
  ValidationResult result;

  validate_required_text(
    result,
    location.location_id,
    "location_id",
    kMaximumLocationIdLength);

  if (
    !location.location_id.empty() &&
    !is_canonical_location_id(
      location.location_id))
  {
    result.add(
      ValidationCode::kInvalidFormat,
      "location_id",
      "location ID is not canonical");
  }

  validate_required_text(
    result,
    location.display_name,
    "display_name",
    kMaximumDisplayNameLength);

  validate_semantic_type(
    result,
    location.semantic_type,
    "semantic_type",
    true);

  validate_identity_aliases(
    result,
    location.location_id,
    location.display_name,
    location.aliases,
    "aliases");

  append(
    result,
    validate_map_context(location.map));

  append(
    result,
    validate_pose(
      location.approach_pose,
      "approach_pose"));

  if (location.confirmation_pose.has_value()) {
    append(
      result,
      validate_pose(
        location.confirmation_pose.value(),
        "confirmation_pose"));
  }

  if (location.tag_pose_map.has_value()) {
    append(
      result,
      validate_pose(
        location.tag_pose_map.value(),
        "tag_pose_map"));
  }

  append(
    result,
    validate_tag_binding(location.tag));

  return result;
}


ValidationResult validate_candidate_draft(
  const CandidateDraft & candidate)
{
  ValidationResult result;

  validate_required_text(
    result,
    candidate.candidate_id,
    "candidate_id",
    kMaximumCandidateIdLength);

  append(
    result,
    validate_map_context(candidate.map));

  append(
    result,
    validate_tag_binding(candidate.tag));

  append(
    result,
    validate_pose(
      candidate.tag_pose_map,
      "tag_pose_map"));

  if (!finite(candidate.detection_quality)) {
    result.add(
      ValidationCode::kNonFiniteNumber,
      "detection_quality",
      "detection quality must be finite");
  } else if (
    candidate.detection_quality < 0.0 ||
    candidate.detection_quality > 1.0)
  {
    result.add(
      ValidationCode::kOutOfRange,
      "detection_quality",
      "detection quality must be within [0, 1]");
  }

  if (candidate.accepted_observations == 0U) {
    result.add(
      ValidationCode::kOutOfRange,
      "accepted_observations",
      "at least one observation is required");
  }

  validate_non_negative_finite(
    result,
    candidate.position_stddev_m,
    "position_stddev_m");

  validate_non_negative_finite(
    result,
    candidate.yaw_stddev_rad,
    "yaw_stddev_rad");

  if (candidate.approach_pose.has_value()) {
    append(
      result,
      validate_pose(
        candidate.approach_pose.value(),
        "approach_pose"));
  }

  if (
    candidate.confirmation_pose.has_value())
  {
    append(
      result,
      validate_pose(
        candidate.confirmation_pose.value(),
        "confirmation_pose"));
  }

  if (
    !candidate.suggested_location_id.empty() &&
    !is_canonical_location_id(
      candidate.suggested_location_id))
  {
    result.add(
      ValidationCode::kInvalidFormat,
      "suggested_location_id",
      "suggested location ID is not canonical");
  }

  validate_optional_text(
    result,
    candidate.suggested_display_name,
    "suggested_display_name",
    kMaximumDisplayNameLength);

  validate_semantic_type(
    result,
    candidate.suggested_semantic_type,
    "suggested_semantic_type",
    false);

  validate_identity_aliases(
    result,
    candidate.suggested_location_id,
    candidate.suggested_display_name,
    candidate.suggested_aliases,
    "suggested_aliases");

  return result;
}

}  // namespace savo_locations
