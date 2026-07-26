#pragma once

#include "savo_mapping/frontier_detector.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace savo_mapping::frontier
{

struct FrontierSelectorConfig
{
  double information_gain_weight{1.0};
  double distance_weight{1.0};
  double minimum_information_gain_m2{0.0};
  double maximum_distance_m{0.0};
};

struct FrontierSelection
{
  std::size_t frontier_index{0};
  double distance_m{0.0};
  double score{0.0};
};

std::string validate_frontier_selector_config(
  const FrontierSelectorConfig & config);

class FrontierSelector
{
public:
  explicit FrontierSelector(
    FrontierSelectorConfig config = {});

  const FrontierSelectorConfig & config() const noexcept;

  std::optional<FrontierSelection> select(
    const std::vector<FrontierCluster> & frontiers,
    double robot_x_m,
    double robot_y_m) const;

private:
  FrontierSelectorConfig config_;
};

}  // namespace savo_mapping::frontier
