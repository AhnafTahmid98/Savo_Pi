#include "savo_locations/types.hpp"

namespace savo_locations
{

std::string_view to_string(
  const CandidateState state) noexcept
{
  switch (state) {
    case CandidateState::kPendingReview:
      return "pending_review";

    case CandidateState::kApproved:
      return "approved";

    case CandidateState::kRejected:
      return "rejected";

    case CandidateState::kUnknown:
    default:
      return "unknown";
  }
}


std::string_view to_string(
  const LocationState state) noexcept
{
  switch (state) {
    case LocationState::kApproved:
      return "approved";

    case LocationState::kRetired:
      return "retired";

    case LocationState::kUnknown:
    default:
      return "unknown";
  }
}


std::string_view to_string(
  const SemanticType type) noexcept
{
  switch (type) {
    case SemanticType::kRoom:
      return "room";

    case SemanticType::kCorridor:
      return "corridor";

    case SemanticType::kReception:
      return "reception";

    case SemanticType::kLaboratory:
      return "laboratory";

    case SemanticType::kElevator:
      return "elevator";

    case SemanticType::kDoorway:
      return "doorway";

    case SemanticType::kStairwell:
      return "stairwell";

    case SemanticType::kRestroom:
      return "restroom";

    case SemanticType::kOffice:
      return "office";

    case SemanticType::kClassroom:
      return "classroom";

    case SemanticType::kServicePoint:
      return "service_point";

    case SemanticType::kChargingStation:
      return "charging_station";

    case SemanticType::kOther:
      return "other";

    case SemanticType::kUnknown:
    default:
      return "unknown";
  }
}


std::optional<SemanticType> semantic_type_from_string(
  const std::string_view value) noexcept
{
  if (value == "room") {
    return SemanticType::kRoom;
  }

  if (value == "corridor") {
    return SemanticType::kCorridor;
  }

  if (value == "reception") {
    return SemanticType::kReception;
  }

  if (value == "laboratory") {
    return SemanticType::kLaboratory;
  }

  if (value == "elevator") {
    return SemanticType::kElevator;
  }

  if (value == "doorway") {
    return SemanticType::kDoorway;
  }

  if (value == "stairwell") {
    return SemanticType::kStairwell;
  }

  if (value == "restroom") {
    return SemanticType::kRestroom;
  }

  if (value == "office") {
    return SemanticType::kOffice;
  }

  if (value == "classroom") {
    return SemanticType::kClassroom;
  }

  if (value == "service_point") {
    return SemanticType::kServicePoint;
  }

  if (value == "charging_station") {
    return SemanticType::kChargingStation;
  }

  if (value == "other") {
    return SemanticType::kOther;
  }

  return std::nullopt;
}


bool is_terminal(
  const CandidateState state) noexcept
{
  return
    state == CandidateState::kApproved ||
    state == CandidateState::kRejected;
}


bool is_navigable(
  const LocationState state,
  const bool enabled) noexcept
{
  return
    state == LocationState::kApproved &&
    enabled;
}

}  // namespace savo_locations
