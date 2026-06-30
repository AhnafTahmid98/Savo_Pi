#include "savo_vo/tracking_quality.hpp"

#include <algorithm>
#include <sstream>

namespace savo_vo
{

TrackingQualityEstimator::TrackingQualityEstimator(
  const int min_features,
  const int good_features_target,
  const double min_score)
: min_features_(std::max(1, min_features)),
  good_features_target_(std::max(1, good_features_target)),
  min_score_(std::clamp(min_score, 0.0, 1.0))
{
}

TrackingQuality TrackingQualityEstimator::evaluate(
  const int detected_features,
  const int tracked_features) const
{
  TrackingQuality quality;
  quality.feature_count = std::max(0, detected_features);
  quality.tracked_count = std::max(0, tracked_features);
  quality.score = compute_tracking_score(
    quality.feature_count,
    quality.tracked_count,
    good_features_target_);

  if (quality.feature_count <= 0) {
    quality.state = VOTrackingState::kWaitingForReference;
  } else if (quality.tracked_count < min_features_) {
    quality.state = VOTrackingState::kRejected;
  } else if (quality.score < min_score_) {
    quality.state = VOTrackingState::kLost;
  } else {
    quality.state = VOTrackingState::kTracking;
  }

  quality.message = build_tracking_message(
    quality.state,
    quality.feature_count,
    quality.tracked_count,
    quality.score);

  return quality;
}

double compute_tracking_score(
  const int detected_features,
  const int tracked_features,
  const int good_features_target)
{
  const int safe_detected = std::max(0, detected_features);
  const int safe_tracked = std::max(0, tracked_features);
  const int safe_target = std::max(1, good_features_target);

  if (safe_detected == 0 || safe_tracked == 0) {
    return 0.0;
  }

  const double retention_score =
    static_cast<double>(safe_tracked) / static_cast<double>(safe_detected);

  const double target_score =
    static_cast<double>(safe_tracked) / static_cast<double>(safe_target);

  const double combined_score = 0.60 * retention_score + 0.40 * target_score;

  return std::clamp(combined_score, 0.0, 1.0);
}

bool is_tracking_usable(
  const TrackingQuality & quality,
  const double min_score,
  const int min_features)
{
  return quality.is_good(
    std::clamp(min_score, 0.0, 1.0),
    std::max(1, min_features));
}

std::string build_tracking_message(
  const VOTrackingState state,
  const int detected_features,
  const int tracked_features,
  const double score)
{
  std::ostringstream stream;
  stream << tracking_state_to_string(state)
         << ": detected=" << std::max(0, detected_features)
         << ", tracked=" << std::max(0, tracked_features)
         << ", score=" << score;

  return stream.str();
}

}  // namespace savo_vo
