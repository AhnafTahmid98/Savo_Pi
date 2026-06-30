#pragma once

#include <string>

#include "savo_vo/vo_types.hpp"

namespace savo_vo
{

class TrackingQualityEstimator
{
public:
  TrackingQualityEstimator(
    int min_features,
    int good_features_target,
    double min_score);

  TrackingQuality evaluate(
    int detected_features,
    int tracked_features) const;

private:
  int min_features_{80};
  int good_features_target_{300};
  double min_score_{0.35};
};

double compute_tracking_score(
  int detected_features,
  int tracked_features,
  int good_features_target);

bool is_tracking_usable(
  const TrackingQuality & quality,
  double min_score,
  int min_features);

std::string build_tracking_message(
  VOTrackingState state,
  int detected_features,
  int tracked_features,
  double score);

}  // namespace savo_vo
