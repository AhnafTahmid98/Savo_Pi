#pragma once

#include "savo_mapping/saved_map_contract.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace savo_mapping::quality
{

inline constexpr char kQualityStateTopic[] =
  "/savo_mapping/map_quality/state";

inline constexpr char kQualityEvaluationTopic[] =
  "/savo_mapping/map_quality/evaluation";

inline constexpr char kNavigationHandoffTopic[] =
  "/savo_mapping/navigation_handoff";

inline constexpr char kEvaluateService[] =
  "/savo_mapping/map_quality/evaluate";

inline constexpr char kApproveHandoffService[] =
  "/savo_mapping/navigation_handoff/approve";

inline constexpr char kRevokeHandoffService[] =
  "/savo_mapping/navigation_handoff/revoke";

struct QualityPolicy
{
  std::size_t min_width_cells{20};
  std::size_t min_height_cells{20};

  std::size_t min_known_cells{500};
  std::size_t min_free_cells{400};
  std::size_t min_occupied_cells{10};

  double min_resolution_m{0.02};
  double max_resolution_m{0.10};

  double min_known_ratio{0.20};
  double min_largest_free_component_ratio{0.85};
};

struct QualityMetrics
{
  std::size_t width{0};
  std::size_t height{0};
  std::size_t total_cells{0};

  std::size_t known_cells{0};
  std::size_t unknown_cells{0};
  std::size_t free_cells{0};
  std::size_t occupied_cells{0};

  std::size_t largest_free_component_cells{0};

  double resolution_m{0.0};
  double mapped_area_m2{0.0};
  double free_area_m2{0.0};

  double known_ratio{0.0};
  double unknown_ratio{0.0};
  double free_ratio{0.0};
  double occupied_ratio_of_known{0.0};
  double largest_free_component_ratio{0.0};
};

struct QualityEvaluation
{
  bool evaluated{false};
  bool passed{false};

  std::string map_id;
  std::string frame_id{"map"};
  std::string reason{"not_evaluated"};

  std::filesystem::path report_path;

  QualityPolicy policy;
  QualityMetrics metrics;

  std::vector<std::string> failed_checks;
};

struct NavigationHandoff
{
  int contract_version{1};

  bool ready{false};
  bool approved{false};

  std::string map_id;
  std::string frame_id{"map"};
  std::string reason{"not_evaluated"};

  std::filesystem::path map_yaml;
  std::filesystem::path quality_report;
};

QualityEvaluation evaluate_occupancy_grid(
  const std::string & map_id,
  const std::string & frame_id,
  const nav_msgs::msg::OccupancyGrid & grid,
  const QualityPolicy & policy);

QualityEvaluation evaluate_saved_map_session(
  const session::SavedMapVerification & verification,
  const QualityPolicy & policy);

void persist_quality_evaluation(
  const session::SavedMapVerification & verification,
  const QualityEvaluation & evaluation);

NavigationHandoff read_navigation_handoff(
  const session::SavedMapVerification & verification);

NavigationHandoff set_navigation_handoff(
  const session::SavedMapVerification & verification,
  bool approved,
  const std::string & reason);

std::string quality_evaluation_to_json(
  const QualityEvaluation & evaluation);

std::string navigation_handoff_to_json(
  const NavigationHandoff & handoff);

}  // namespace savo_mapping::quality
