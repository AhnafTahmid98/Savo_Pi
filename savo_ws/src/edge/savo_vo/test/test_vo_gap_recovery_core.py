# Copyright 2026 Ahnaf Tahmid

"""ROS-independent regression for transient VO gap health recovery."""

import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_transient_gap_health_recovers_after_fresh_status_and_odom(
    tmp_path: Path,
) -> None:
    """Compile pure health state and exercise reject-to-healthy recovery."""
    source = tmp_path / "vo_gap_recovery_test.cpp"
    binary = tmp_path / "vo_gap_recovery_test"
    source.write_text(
        r'''
#include <cassert>
#include <string>

#include "savo_vo/vo_health_state.hpp"

int main()
{
  savo_vo::VOHealthState state;
  state.has_status = true;
  state.last_status_time_s = 9.6;
  state.status_text =
    "rejected: invalid RGB-D frame interval; reference reseeded";
  state.has_odom = true;
  state.last_odom_time_s = 9.0;
  assert(savo_vo::evaluate_vo_health(state, 10.0, 0.5).rfind(
    "degraded:", 0U) == 0U);

  state.last_status_time_s = 9.9;
  state.status_text = "tracking accepted=true";
  state.last_odom_time_s = 9.9;
  assert(savo_vo::evaluate_vo_health(state, 10.0, 0.5) ==
    "ok: tracking accepted=true");
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
        str(PACKAGE / "src/vo_health_state.cpp"),
        "-o",
        str(binary),
    ]
    compiled = subprocess.run(
        command, check=False, capture_output=True, text=True
    )
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    executed = subprocess.run(
        [str(binary)], check=False, capture_output=True, text=True
    )
    assert executed.returncode == 0, executed.stdout + executed.stderr
