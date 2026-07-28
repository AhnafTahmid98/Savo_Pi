// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace savo_perception
{

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ObstacleCloudFilterConfig
{
  double min_range_m{0.20};
  double max_range_m{3.00};
  double min_height_m{0.05};
  double max_height_m{1.60};
  double voxel_size_m{0.05};

  bool self_filter_enabled{true};
  double self_min_x_m{-0.30};
  double self_max_x_m{0.30};
  double self_min_y_m{-0.26};
  double self_max_y_m{0.26};
  double self_min_z_m{-0.10};
  double self_max_z_m{0.70};

  std::size_t max_output_points{100000U};
};

struct ObstacleCloudFilterStats
{
  std::size_t input_points{0U};
  std::size_t finite_points{0U};
  std::size_t range_rejected{0U};
  std::size_t height_rejected{0U};
  std::size_t self_rejected{0U};
  std::size_t voxel_rejected{0U};
  std::size_t output_points{0U};
  bool output_limited{false};
};

struct ObstacleCloudFilterResult
{
  std::vector<PointXYZ> points;
  ObstacleCloudFilterStats stats;
};

std::string validate_obstacle_cloud_filter_config(
  const ObstacleCloudFilterConfig & config);

ObstacleCloudFilterResult filter_obstacle_cloud(
  const std::vector<PointXYZ> & points,
  const ObstacleCloudFilterConfig & config);

}  // namespace savo_perception
