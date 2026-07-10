#include "savo_mapping/mapping_types.hpp"

#include <cmath>

namespace savo_mapping
{

bool is_valid_pose(const Pose2D & pose)
{
  return std::isfinite(pose.x_m) &&
         std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool is_quality_score_valid(double score)
{
  return std::isfinite(score) && score >= 0.0 && score <= 1.0;
}

bool is_quality_score_navigation_ready(double score, double threshold)
{
  return is_quality_score_valid(score) &&
         is_quality_score_valid(threshold) &&
         score >= threshold;
}

bool has_saved_map_paths(const SavedMapInfo & info)
{
  return !info.map_name.empty() &&
         !info.map_yaml_path.empty() &&
         !info.map_image_path.empty();
}

bool is_semantic_observation_valid(const SemanticLandmarkObservation & observation)
{
  return !observation.label.empty() &&
         !observation.source.empty() &&
         is_valid_pose(observation.map_pose) &&
         is_quality_score_valid(observation.confidence);
}

}  // namespace savo_mapping
