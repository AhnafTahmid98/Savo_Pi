# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""ROS-independent compile and behavior checks for the obstacle-cloud core."""

import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[2]


def test_incremental_filter_and_processing_gate(tmp_path):
    """Compile the pure core and verify filtering/gating without ROS."""
    source = tmp_path / "obstacle_cloud_core.cpp"
    binary = tmp_path / "obstacle_cloud_core"
    source.write_text(
        r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "savo_perception/obstacle_cloud_filter.hpp"

int main()
{
  using namespace savo_perception;

  ObstacleCloudFilterConfig config;
  config.self_filter_enabled = false;

  const std::vector<PointXYZ> points{
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2},
    {0.1, 0.0, 0.2},
    {1.0, 0.0, 0.3},
    {1.01, 0.0, 0.3},
    {1.2, 0.0, 1.7}};

  const auto vector_result = filter_obstacle_cloud(points, config);
  ObstacleCloudFilterAccumulator incremental(config);
  for (const auto & point : points) {
    incremental.consume(point);
  }
  const auto & incremental_result = incremental.result();

  assert(vector_result.points.size() == incremental_result.points.size());
  assert(vector_result.stats.input_points == incremental_result.stats.input_points);
  assert(vector_result.stats.finite_points == incremental_result.stats.finite_points);
  assert(vector_result.stats.range_rejected == incremental_result.stats.range_rejected);
  assert(vector_result.stats.height_rejected == incremental_result.stats.height_rejected);
  assert(vector_result.stats.voxel_rejected == incremental_result.stats.voxel_rejected);
  assert(vector_result.stats.output_points == incremental_result.stats.output_points);

  ObstacleCloudProcessingGate gate(10.0);
  assert(gate.should_process(100.0));
  assert(!gate.should_process(100.05));
  assert(gate.should_process(100.101));
  assert(gate.should_process(1.0));
  assert(!gate.should_process(1.05));
  assert(gate.should_process(1.101));

  return 0;
}
''',
        encoding="utf-8",
    )

    command = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-Werror",
        "-I",
        str(PACKAGE / "include"),
        str(source),
        str(PACKAGE / "src/filtering/obstacle_cloud_filter.cpp"),
        "-o",
        str(binary),
    ]
    compiled = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    executed = subprocess.run(
        [str(binary)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert executed.returncode == 0, executed.stdout + executed.stderr
