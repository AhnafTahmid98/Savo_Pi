// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <memory>
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

struct PointCloudStorageLayout
{
  std::size_t width{0U};
  std::size_t height{0U};
  std::size_t point_step{0U};
  std::size_t row_step{0U};
  std::size_t data_size{0U};
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

class ObstacleCloudFilterAccumulator
{
public:
  explicit ObstacleCloudFilterAccumulator(
    const ObstacleCloudFilterConfig & config);
  ~ObstacleCloudFilterAccumulator();

  ObstacleCloudFilterAccumulator(
    const ObstacleCloudFilterAccumulator &) = delete;
  ObstacleCloudFilterAccumulator & operator=(
    const ObstacleCloudFilterAccumulator &) = delete;

  void reset();
  void reject_non_finite_input();
  void consume(const PointXYZ & point);

  [[nodiscard]] const ObstacleCloudFilterResult & result() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class ObstacleCloudProcessingGate
{
public:
  explicit ObstacleCloudProcessingGate(double max_processing_hz);

  [[nodiscard]] bool should_process(double monotonic_time_s);
  void reset();

private:
  double minimum_interval_s_{0.0};
  double last_selected_time_s_{0.0};
  bool have_selected_time_{false};
};

std::string validate_obstacle_cloud_filter_config(
  const ObstacleCloudFilterConfig & config);

std::string validate_point_cloud_storage_layout(
  const PointCloudStorageLayout & layout);

ObstacleCloudFilterResult filter_obstacle_cloud(
  const std::vector<PointXYZ> & points,
  const ObstacleCloudFilterConfig & config);

}  // namespace savo_perception
