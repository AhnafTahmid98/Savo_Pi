#pragma once

#include <cstdint>
#include <string>

namespace savo_mapping
{

struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct MapQualityMetrics
{
  bool map_received{false};
  bool scan_received{false};
  bool tf_ok{false};
  bool odom_ok{false};

  double quality_score{0.0};
  double free_ratio{0.0};
  double occupied_ratio{0.0};
  double unknown_ratio{1.0};
  double map_area_m2{0.0};

  std::uint64_t map_updates{0};
  std::uint64_t scan_updates{0};
};

struct SavedMapInfo
{
  std::string map_name{};
  std::string map_yaml_path{};
  std::string map_image_path{};
  std::string quality_report_path{};
  std::string session_report_path{};
  std::string semantic_landmarks_path{};
};

struct SemanticLandmarkObservation
{
  std::string label{};
  std::string source{};
  Pose2D map_pose{};
  double confidence{0.0};
};

bool is_valid_pose(const Pose2D & pose);
bool is_quality_score_valid(double score);
bool is_quality_score_navigation_ready(double score, double threshold);
bool has_saved_map_paths(const SavedMapInfo & info);
bool is_semantic_observation_valid(const SemanticLandmarkObservation & observation);

}  // namespace savo_mapping
