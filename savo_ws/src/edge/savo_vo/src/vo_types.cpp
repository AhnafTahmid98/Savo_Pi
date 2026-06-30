#include "savo_vo/vo_types.hpp"

namespace savo_vo
{

bool CameraIntrinsics::is_valid() const
{
  return fx > 0.0 &&
         fy > 0.0 &&
         width > 0 &&
         height > 0;
}

bool TrackingQuality::is_good(
  double minimum_score,
  int minimum_features) const
{
  return state == VOTrackingState::kTracking &&
         score >= minimum_score &&
         tracked_count >= minimum_features;
}

std::string health_level_to_string(const VOHealthLevel level)
{
  switch (level) {
    case VOHealthLevel::kWaiting:
      return "waiting";
    case VOHealthLevel::kOk:
      return "ok";
    case VOHealthLevel::kStale:
      return "stale";
    case VOHealthLevel::kDegraded:
      return "degraded";
    case VOHealthLevel::kError:
      return "error";
  }

  return "unknown";
}

std::string tracking_state_to_string(const VOTrackingState state)
{
  switch (state) {
    case VOTrackingState::kWaitingForReference:
      return "waiting_for_reference";
    case VOTrackingState::kTracking:
      return "tracking";
    case VOTrackingState::kRejected:
      return "rejected";
    case VOTrackingState::kLost:
      return "lost";
    case VOTrackingState::kError:
      return "error";
  }

  return "unknown";
}

}  // namespace savo_vo
