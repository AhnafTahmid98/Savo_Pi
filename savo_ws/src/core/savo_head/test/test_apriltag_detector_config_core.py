import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(code: str) -> None:
    with tempfile.TemporaryDirectory(prefix="savo_head_apriltag_detector_") as temp:
        temp_path = Path(temp)
        source = temp_path / "test.cpp"
        executable = temp_path / "test"
        source.write_text(code, encoding="utf-8")

        command = [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-I",
            str(ROOT / "include"),
            str(source),
            str(ROOT / "src/core/apriltag_detector_config.cpp"),
            "-o",
            str(executable),
        ]
        result = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr

        result = subprocess.run(
            [str(executable)],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr


def test_apriltag_detector_configuration_contract() -> None:
    compile_and_run(
        r'''
#include <cassert>
#include <string>
#include <vector>

#include "savo_head/core/apriltag_detector_config.hpp"

int main()
{
  using savo_head::AprilTagDetectorConfig;

  AprilTagDetectorConfig config;
  assert(savo_head::validate_apriltag_detector_config(config).empty());
  assert(config.family == "tag36h11");
  assert(config.tag_size_m == 0.140);

  config.allowed_tag_ids = {0, 1, 2, 3, 4, 5};
  for (int tag_id = 0; tag_id <= 5; ++tag_id) {
    assert(savo_head::is_allowed_apriltag_id(config, tag_id));
  }
  assert(!savo_head::is_allowed_apriltag_id(config, 6));
  assert(!savo_head::is_allowed_apriltag_id(config, -1));

  config.family = "tag25h9";
  assert(!savo_head::validate_apriltag_detector_config(config).empty());

  config.family = "tag36h11";
  config.tag_size_m = 0.0;
  assert(!savo_head::validate_apriltag_detector_config(config).empty());

  config.tag_size_m = -0.140;
  assert(!savo_head::validate_apriltag_detector_config(config).empty());

  assert(savo_head::normalized_detection_quality(50.0, 100.0) == 0.5);
  assert(savo_head::normalized_detection_quality(150.0, 100.0) == 1.0);
  return 0;
}
'''
    )
