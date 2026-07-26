#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace savo_mapping::frontier
{

inline constexpr std::int8_t UNKNOWN_CELL = -1;

struct GridCell
{
  std::size_t x{0};
  std::size_t y{0};
};

bool operator==(const GridCell & left, const GridCell & right);
bool operator!=(const GridCell & left, const GridCell & right);

struct OccupancyGrid
{
  std::size_t width{0};
  std::size_t height{0};
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  std::vector<std::int8_t> cells;
};

struct FrontierDetectorConfig
{
  std::int8_t free_threshold{0};
  std::size_t minimum_cluster_size{1};
  bool cluster_diagonally{true};
};

struct FrontierCluster
{
  std::vector<GridCell> cells;
  GridCell representative;
  double centroid_x_m{0.0};
  double centroid_y_m{0.0};
  double information_gain_m2{0.0};
};

std::string validate_occupancy_grid(const OccupancyGrid & grid);

std::string validate_frontier_detector_config(
  const FrontierDetectorConfig & config);

class FrontierDetector
{
public:
  explicit FrontierDetector(
    FrontierDetectorConfig config = {});

  const FrontierDetectorConfig & config() const noexcept;

  std::vector<FrontierCluster> detect(
    const OccupancyGrid & grid) const;

private:
  FrontierDetectorConfig config_;
};

}  // namespace savo_mapping::frontier
