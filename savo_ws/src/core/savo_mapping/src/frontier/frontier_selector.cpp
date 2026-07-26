#include "savo_mapping/frontier_selector.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace savo_mapping::frontier
{
namespace
{

bool valid_frontier(const FrontierCluster & frontier)
{
  return !frontier.cells.empty() &&
         std::isfinite(frontier.centroid_x_m) &&
         std::isfinite(frontier.centroid_y_m) &&
         std::isfinite(frontier.information_gain_m2) &&
         frontier.information_gain_m2 >= 0.0;
}

bool better_candidate(
  const FrontierSelection & candidate,
  const FrontierCluster & candidate_frontier,
  const FrontierSelection & current,
  const FrontierCluster & current_frontier)
{
  constexpr double epsilon = 1.0e-12;

  if (candidate.score > current.score + epsilon) {
    return true;
  }

  if (current.score > candidate.score + epsilon) {
    return false;
  }

  if (candidate_frontier.information_gain_m2 >
    current_frontier.information_gain_m2 + epsilon)
  {
    return true;
  }

  if (current_frontier.information_gain_m2 >
    candidate_frontier.information_gain_m2 + epsilon)
  {
    return false;
  }

  if (candidate.distance_m + epsilon < current.distance_m) {
    return true;
  }

  if (current.distance_m + epsilon < candidate.distance_m) {
    return false;
  }

  if (candidate_frontier.representative.y !=
    current_frontier.representative.y)
  {
    return candidate_frontier.representative.y <
           current_frontier.representative.y;
  }

  if (candidate_frontier.representative.x !=
    current_frontier.representative.x)
  {
    return candidate_frontier.representative.x <
           current_frontier.representative.x;
  }

  return candidate.frontier_index < current.frontier_index;
}

}  // namespace

std::string validate_frontier_selector_config(
  const FrontierSelectorConfig & config)
{
  if (!std::isfinite(config.information_gain_weight) ||
    config.information_gain_weight < 0.0)
  {
    return "information_gain_weight_must_be_nonnegative";
  }

  if (!std::isfinite(config.distance_weight) ||
    config.distance_weight < 0.0)
  {
    return "distance_weight_must_be_nonnegative";
  }

  if (!std::isfinite(config.minimum_information_gain_m2) ||
    config.minimum_information_gain_m2 < 0.0)
  {
    return "minimum_information_gain_must_be_nonnegative";
  }

  if (!std::isfinite(config.maximum_distance_m) ||
    config.maximum_distance_m < 0.0)
  {
    return "maximum_distance_must_be_nonnegative";
  }

  return {};
}

FrontierSelector::FrontierSelector(
  FrontierSelectorConfig config)
: config_(config)
{
  const std::string error =
    validate_frontier_selector_config(config_);

  if (!error.empty()) {
    throw std::invalid_argument(
            "invalid frontier selector configuration: " + error);
  }
}

const FrontierSelectorConfig & FrontierSelector::config() const noexcept
{
  return config_;
}

std::optional<FrontierSelection> FrontierSelector::select(
  const std::vector<FrontierCluster> & frontiers,
  double robot_x_m,
  double robot_y_m) const
{
  if (!std::isfinite(robot_x_m) || !std::isfinite(robot_y_m)) {
    throw std::invalid_argument("robot pose must be finite");
  }

  std::optional<FrontierSelection> best;

  for (std::size_t index = 0; index < frontiers.size(); ++index) {
    const FrontierCluster & frontier = frontiers.at(index);

    if (!valid_frontier(frontier)) {
      continue;
    }

    if (frontier.information_gain_m2 <
      config_.minimum_information_gain_m2)
    {
      continue;
    }

    const double distance_m = std::hypot(
      frontier.centroid_x_m - robot_x_m,
      frontier.centroid_y_m - robot_y_m);

    if (config_.maximum_distance_m > 0.0 &&
      distance_m > config_.maximum_distance_m)
    {
      continue;
    }

    const double score =
      config_.information_gain_weight *
      frontier.information_gain_m2 -
      config_.distance_weight * distance_m;

    const FrontierSelection candidate{
      index,
      distance_m,
      score,
    };

    if (!best.has_value() ||
      better_candidate(
          candidate,
          frontier,
          *best,
          frontiers.at(best->frontier_index)))
    {
      best = candidate;
    }
  }

  return best;
}

}  // namespace savo_mapping::frontier
