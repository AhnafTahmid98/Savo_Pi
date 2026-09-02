// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_perception/obstacle_cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace savo_perception
{

namespace
{

bool finite(const double value)
{
  return std::isfinite(value);
}

bool checked_multiply(
  const std::size_t left,
  const std::size_t right,
  std::size_t & result)
{
  if (
    right != 0U &&
    left > std::numeric_limits<std::size_t>::max() / right)
  {
    return false;
  }

  result = left * right;
  return true;
}

bool point_is_finite(const PointXYZ & point)
{
  return
    finite(point.x) &&
    finite(point.y) &&
    finite(point.z);
}

bool point_is_in_self_box(
  const PointXYZ & point,
  const ObstacleCloudFilterConfig & config)
{
  return
    point.x >= config.self_min_x_m &&
    point.x <= config.self_max_x_m &&
    point.y >= config.self_min_y_m &&
    point.y <= config.self_max_y_m &&
    point.z >= config.self_min_z_m &&
    point.z <= config.self_max_z_m;
}

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return
      x == other.x &&
      y == other.y &&
      z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto x = std::hash<std::int64_t>{}(key.x);
    const auto y = std::hash<std::int64_t>{}(key.y);
    const auto z = std::hash<std::int64_t>{}(key.z);

    return x ^ (y << 1U) ^ (z << 2U);
  }
};

VoxelKey make_voxel_key(
  const PointXYZ & point,
  const double inverse_voxel_size)
{
  return {
    static_cast<std::int64_t>(
      std::floor(point.x * inverse_voxel_size)),
    static_cast<std::int64_t>(
      std::floor(point.y * inverse_voxel_size)),
    static_cast<std::int64_t>(
      std::floor(point.z * inverse_voxel_size))};
}

constexpr std::size_t kInitialCandidateReserve = 16384U;

}  // namespace

std::string validate_point_cloud_storage_layout(
  const PointCloudStorageLayout & layout)
{
  if (layout.point_step == 0U) {
    return "malformed_pointcloud_layout";
  }

  std::size_t minimum_row_step = 0U;

  if (!checked_multiply(
      layout.point_step,
      layout.width,
      minimum_row_step))
  {
    return "malformed_pointcloud_layout";
  }

  // The node iterates points linearly, so only one row may have trailing
  // storage that is not part of the declared width.
  if (
    layout.row_step < minimum_row_step ||
    (layout.height != 1U && layout.row_step != minimum_row_step))
  {
    return "malformed_pointcloud_layout";
  }

  std::size_t declared_data_size = 0U;

  if (!checked_multiply(
      layout.row_step,
      layout.height,
      declared_data_size))
  {
    return "malformed_pointcloud_layout";
  }

  if (
    layout.height == 1U &&
    (layout.data_size < minimum_row_step ||
    layout.data_size > declared_data_size))
  {
    return "malformed_pointcloud_layout";
  }

  if (
    layout.height != 1U &&
    layout.data_size != declared_data_size)
  {
    return "malformed_pointcloud_layout";
  }

  return {};
}

std::string validate_obstacle_cloud_filter_config(
  const ObstacleCloudFilterConfig & config)
{
  if (
    !finite(config.min_range_m) ||
    !finite(config.max_range_m) ||
    !finite(config.min_height_m) ||
    !finite(config.max_height_m) ||
    !finite(config.voxel_size_m) ||
    !finite(config.self_min_x_m) ||
    !finite(config.self_max_x_m) ||
    !finite(config.self_min_y_m) ||
    !finite(config.self_max_y_m) ||
    !finite(config.self_min_z_m) ||
    !finite(config.self_max_z_m))
  {
    return "non_finite_configuration";
  }

  if (config.min_range_m < 0.0) {
    return "negative_minimum_range";
  }

  if (config.max_range_m <= config.min_range_m) {
    return "invalid_range_bounds";
  }

  if (config.max_height_m <= config.min_height_m) {
    return "invalid_height_bounds";
  }

  if (config.voxel_size_m <= 0.0) {
    return "invalid_voxel_size";
  }

  if (
    config.self_max_x_m <= config.self_min_x_m ||
    config.self_max_y_m <= config.self_min_y_m ||
    config.self_max_z_m <= config.self_min_z_m)
  {
    return "invalid_self_filter_bounds";
  }

  if (config.max_output_points == 0U) {
    return "zero_maximum_output_points";
  }

  return {};
}

struct ObstacleCloudFilterAccumulator::Impl
{
  explicit Impl(
    const ObstacleCloudFilterConfig & filter_config)
  : config(filter_config),
    minimum_range_squared(
      filter_config.min_range_m * filter_config.min_range_m),
    maximum_range_squared(
      filter_config.max_range_m * filter_config.max_range_m),
    inverse_voxel_size(1.0 / filter_config.voxel_size_m)
  {
    const auto validation =
      validate_obstacle_cloud_filter_config(config);

    if (!validation.empty()) {
      throw std::invalid_argument(validation);
    }

    const auto initial_reserve =
      std::min(config.max_output_points, kInitialCandidateReserve);

    filter_result.points.reserve(initial_reserve);
    occupied_voxels.reserve(initial_reserve);
  }

  ObstacleCloudFilterConfig config;
  double minimum_range_squared;
  double maximum_range_squared;
  double inverse_voxel_size;
  ObstacleCloudFilterResult filter_result;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
};

ObstacleCloudFilterAccumulator::ObstacleCloudFilterAccumulator(
  const ObstacleCloudFilterConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

ObstacleCloudFilterAccumulator::~ObstacleCloudFilterAccumulator() = default;

void ObstacleCloudFilterAccumulator::reset()
{
  impl_->filter_result.points.clear();
  impl_->filter_result.stats = {};
  impl_->occupied_voxels.clear();
}

void ObstacleCloudFilterAccumulator::reject_non_finite_input()
{
  ++impl_->filter_result.stats.input_points;
}

void ObstacleCloudFilterAccumulator::consume(
  const PointXYZ & point)
{
  auto & result = impl_->filter_result;
  const auto & config = impl_->config;

  ++result.stats.input_points;

  if (!point_is_finite(point)) {
    return;
  }

  ++result.stats.finite_points;

  const double horizontal_range_squared =
    point.x * point.x + point.y * point.y;

  if (
    horizontal_range_squared < impl_->minimum_range_squared ||
    horizontal_range_squared > impl_->maximum_range_squared)
  {
    ++result.stats.range_rejected;
    return;
  }

  if (
    point.z < config.min_height_m ||
    point.z > config.max_height_m)
  {
    ++result.stats.height_rejected;
    return;
  }

  if (
    config.self_filter_enabled &&
    point_is_in_self_box(point, config))
  {
    ++result.stats.self_rejected;
    return;
  }

  const auto voxel =
    make_voxel_key(point, impl_->inverse_voxel_size);

  if (impl_->occupied_voxels.find(voxel) != impl_->occupied_voxels.end()) {
    ++result.stats.voxel_rejected;
    return;
  }

  if (result.points.size() >= config.max_output_points) {
    result.stats.output_limited = true;
    return;
  }

  impl_->occupied_voxels.insert(voxel);
  result.points.push_back(point);
  result.stats.output_points = result.points.size();
}

const ObstacleCloudFilterResult &
ObstacleCloudFilterAccumulator::result() const
{
  return impl_->filter_result;
}

ObstacleCloudProcessingGate::ObstacleCloudProcessingGate(
  const double max_processing_hz)
{
  if (!finite(max_processing_hz) || max_processing_hz <= 0.0) {
    throw std::invalid_argument("invalid_max_processing_rate");
  }

  minimum_interval_s_ = 1.0 / max_processing_hz;
}

bool ObstacleCloudProcessingGate::should_process(
  const double monotonic_time_s)
{
  if (!finite(monotonic_time_s)) {
    throw std::invalid_argument("non_finite_processing_time");
  }

  if (
    !have_selected_time_ ||
    monotonic_time_s < last_selected_time_s_ ||
    monotonic_time_s - last_selected_time_s_ >= minimum_interval_s_)
  {
    last_selected_time_s_ = monotonic_time_s;
    have_selected_time_ = true;
    return true;
  }

  return false;
}

void ObstacleCloudProcessingGate::reset()
{
  last_selected_time_s_ = 0.0;
  have_selected_time_ = false;
}

ObstacleCloudFilterResult filter_obstacle_cloud(
  const std::vector<PointXYZ> & points,
  const ObstacleCloudFilterConfig & config)
{
  ObstacleCloudFilterAccumulator accumulator(config);

  for (const auto & point : points) {
    accumulator.consume(point);
  }

  return accumulator.result();
}

}  // namespace savo_perception
