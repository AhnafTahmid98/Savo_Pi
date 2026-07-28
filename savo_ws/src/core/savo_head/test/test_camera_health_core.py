import subprocess
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def run_command(
    command: list[str],
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=PACKAGE_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, (
        "Command failed:\n"
        + " ".join(command)
        + "\n\nSTDOUT:\n"
        + result.stdout
        + "\nSTDERR:\n"
        + result.stderr
    )

    return result


def test_camera_health_core_compiles_and_evaluates(
    tmp_path: Path,
):
    source = tmp_path / "camera_health_core.cpp"
    binary = tmp_path / "camera_health_core"

    source.write_text(
        r'''
#include <cassert>
#include <iostream>
#include <string>

#include "savo_head/core/camera_health.hpp"

int main()
{
  using namespace savo_head;

  CameraHealthConfig config;
  assert(config.valid());

  CameraHealthSnapshot snapshot;
  snapshot.start_time_s = 10.0;
  snapshot.now_s = 10.5;

  /*
   * During startup, no image is a warning.
   */
  auto result = evaluate_camera_health(config, snapshot);

  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "waiting_for_image");
  assert(!result.stream_healthy);

  /*
   * After startup grace, no image is an error.
   */
  snapshot.now_s = 14.0;

  result = evaluate_camera_health(config, snapshot);

  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "image_not_received");

  /*
   * Simulate a healthy 640x480 RGB stream with valid CameraInfo
   * dimensions but no calibration.
   */
  snapshot.image_seen = true;
  snapshot.camera_info_seen = true;

  snapshot.image_receipt_time_s = 14.0;
  snapshot.camera_info_receipt_time_s = 14.0;

  snapshot.image_data_valid = true;

  snapshot.image_width = 640;
  snapshot.image_height = 480;

  snapshot.camera_info_width = 640;
  snapshot.camera_info_height = 480;

  snapshot.image_frame_id = "pi_camera_optical_frame";
  snapshot.camera_info_frame_id = "pi_camera_optical_frame";

  snapshot.image_encoding = "rgb8";

  snapshot.frames_received = 10;
  snapshot.frame_rate_hz = 29.8;

  snapshot.camera_calibrated = false;

  result = evaluate_camera_health(config, snapshot);

  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "camera_uncalibrated");
  assert(result.stream_healthy);
  assert(!result.ready_for_pose_estimation);

  const auto text =
    camera_health_status_text(result, snapshot);

  assert(
    text.find("status=WARN") != std::string::npos);

  assert(
    text.find("reason=camera_uncalibrated") !=
    std::string::npos);

  assert(
    text.find("stream_healthy=true") !=
    std::string::npos);

  assert(
    text.find("ready_for_pose_estimation=false") !=
    std::string::npos);

  /*
   * A calibrated camera becomes ready for AprilTag metric pose
   * estimation.
   */
  snapshot.camera_calibrated = true;

  result = evaluate_camera_health(config, snapshot);

  assert(result.level == CameraHealthLevel::kOk);
  assert(result.reason == "camera_ready");
  assert(result.stream_healthy);
  assert(result.ready_for_pose_estimation);

  /*
   * A previously valid stream that stops updating becomes stale.
   */
  snapshot.now_s = 17.0;

  result = evaluate_camera_health(config, snapshot);

  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "image_stale");

  std::cout << "PASS camera_health_core\n";
  return 0;
}
''',
        encoding="utf-8",
    )

    run_command(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-I",
            str(PACKAGE_ROOT / "include"),
            str(source),
            str(
                PACKAGE_ROOT
                / "src/core/camera_health.cpp"
            ),
            "-o",
            str(binary),
        ]
    )

    result = run_command([str(binary)])

    assert "PASS camera_health_core" in result.stdout


def test_camera_health_config_defaults():
    config_path = (
        PACKAGE_ROOT
        / "config/camera_health.yaml"
    )

    data = yaml.safe_load(
        config_path.read_text(encoding="utf-8")
    )

    params = data[
        "/savo_head/head_camera_status_node"
    ]["ros__parameters"]

    assert (
        params["image_topic"]
        == "/savo_head/camera/image_raw"
    )

    assert (
        params["camera_info_topic"]
        == "/savo_head/camera/camera_info"
    )

    assert (
        params["status_topic"]
        == "/savo_head/camera/status"
    )

    assert params["expected_width"] == 640
    assert params["expected_height"] == 480

    assert (
        params["expected_frame_id"]
        == "pi_camera_optical_frame"
    )

    assert params["expected_encoding"] == "rgb8"

    assert (
        params["require_calibration_for_pose"]
        is True
    )
