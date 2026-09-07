# Copyright 2026 Ahnaf Tahmid

"""ROS-independent behavior test for bounded latest-frame selection."""

import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_latest_frame_selector_and_interval_guard(tmp_path: Path) -> None:
    """Compile the pure selector and prove overwrite-only queue behavior."""
    source = tmp_path / "latest_frame_selector_test.cpp"
    binary = tmp_path / "latest_frame_selector_test"
    source.write_text(
        r'''
#include <cassert>
#include <limits>

#include "savo_vo/latest_frame_selector.hpp"

void verify_source_rate(const double source_rate_hz)
{
  constexpr double processing_rate_hz = 12.0;
  savo_vo::LatestFrameSelector selector;
  double next_source_stamp_s = 0.0;
  double previous_processed_stamp_s = 0.0;
  bool have_previous = false;

  for (int tick = 0; tick <= 24; ++tick) {
    const double processing_time_s =
      static_cast<double>(tick) / processing_rate_hz;
    while (next_source_stamp_s <= processing_time_s + 1e-9) {
      assert(selector.offer(next_source_stamp_s));
      next_source_stamp_s += 1.0 / source_rate_hz;
    }
    const auto selected = selector.take();
    if (!selected.has_value()) {
      continue;
    }
    if (have_previous) {
      assert(savo_vo::valid_frame_interval(
        previous_processed_stamp_s, selected.value(), 0.20));
    }
    previous_processed_stamp_s = selected.value();
    have_previous = true;
  }
}

int main()
{
  savo_vo::LatestFrameSelector selector;
  for (int index = 1; index <= 1000; ++index) {
    assert(selector.offer(static_cast<double>(index) * 0.01));
  }
  assert(selector.pending_count() == 1U);
  const auto newest = selector.take();
  assert(newest.has_value());
  assert(newest.value() == 10.0);
  assert(selector.pending_count() == 0U);
  assert(!selector.offer(9.99));
  assert(!selector.offer(std::numeric_limits<double>::infinity()));

  assert(savo_vo::valid_frame_interval(1.0, 1.1, 0.20));
  assert(!savo_vo::valid_frame_interval(1.0, 1.0, 0.20));
  assert(!savo_vo::valid_frame_interval(1.0, 1.201, 0.20));

  double reference_stamp_s = 1.0;
  const double post_gap_stamp_s = 1.4;
  assert(!savo_vo::valid_frame_interval(
    reference_stamp_s, post_gap_stamp_s, 0.20));
  reference_stamp_s = post_gap_stamp_s;
  assert(savo_vo::valid_frame_interval(
    reference_stamp_s, post_gap_stamp_s + (1.0 / 15.0), 0.20));

  verify_source_rate(15.0);
  verify_source_rate(20.0);
  verify_source_rate(25.0);
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
        str(PACKAGE / "src/latest_frame_selector.cpp"),
        "-o",
        str(binary),
    ]
    compiled = subprocess.run(command, check=False, capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    executed = subprocess.run(
        [str(binary)], check=False, capture_output=True, text=True
    )
    assert executed.returncode == 0, executed.stdout + executed.stderr
