// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_perception/obstacle_cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
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
  const double voxel_size_m)
{
  return {
    static_cast<std::int64_t>(
      std::floor(point.x / voxel_size_m)),
    static_cast<std::int64_t>(
      std::floor(point.y / voxel_size_m)),
    static_cast<std::int64_t>(
      std::floor(point.z / voxel_size_m))};
}

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

  if (layout.data_size != declared_data_size) {
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

ObstacleCloudFilterResult filter_obstacle_cloud(
  const std::vector<PointXYZ> & points,
  const ObstacleCloudFilterConfig & config)
{
  const auto validation =
    validate_obstacle_cloud_filter_config(config);

  if (!validation.empty()) {
    throw std::invalid_argument(validation);
  }

  ObstacleCloudFilterResult result;
  result.stats.input_points = points.size();

  result.points.reserve(
    std::min(
      points.size(),
      config.max_output_points));

  std::unordered_set<VoxelKey, VoxelKeyHash>
  occupied_voxels;

  occupied_voxels.reserve(points.size());

  for (const auto & point : points) {
    if (!point_is_finite(point)) {
      continue;
    }

    ++result.stats.finite_points;

    const double horizontal_range =
      std::hypot(point.x, point.y);

    if (
      horizontal_range < config.min_range_m ||
      horizontal_range > config.max_range_m)
    {
      ++result.stats.range_rejected;
      continue;
    }

    if (
      point.z < config.min_height_m ||
      point.z > config.max_height_m)
    {
      ++result.stats.height_rejected;
      continue;
    }

    if (
      config.self_filter_enabled &&
      point_is_in_self_box(point, config))
    {
      ++result.stats.self_rejected;
      continue;
    }

    const auto voxel =
      make_voxel_key(point, config.voxel_size_m);

    if (!occupied_voxels.insert(voxel).second) {
      ++result.stats.voxel_rejected;
      continue;
    }

    if (
      result.points.size() >=
      config.max_output_points)
    {
      result.stats.output_limited = true;
      continue;
    }

    result.points.push_back(point);
  }

  result.stats.output_points =
    result.points.size();

  return result;
}

}  // namespace savo_perception
