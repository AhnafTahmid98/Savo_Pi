# Copyright 2026 Ahnaf Tahmid

"""ROS-independent behavior test for bounded front-depth processing."""

import subprocess
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_depth_processing_gate_selects_fresh_samples(tmp_path: Path) -> None:
    """Compile the pure gate and prove it never accumulates skipped input."""
    source = tmp_path / "depth_processing_gate_test.cpp"
    binary = tmp_path / "depth_processing_gate_test"
    source.write_text(
        r'''
#include <cassert>

#include "savo_realsense/depth_processing_gate.hpp"

int main()
{
  savo_realsense::DepthProcessingGate gate(15.0);
  assert(gate.should_process(1.000));
  assert(!gate.should_process(1.033));
  assert(!gate.should_process(1.066));
  assert(gate.should_process(1.067));
  assert(gate.should_process(0.500));
  assert(!gate.should_process(0.520));
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
        str(PACKAGE / "src/depth_processing_gate.cpp"),
        "-o",
        str(binary),
    ]
    compiled = subprocess.run(command, check=False, capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    executed = subprocess.run(
        [str(binary)], check=False, capture_output=True, text=True
    )
    assert executed.returncode == 0, executed.stdout + executed.stderr
