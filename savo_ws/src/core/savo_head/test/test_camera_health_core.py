import subprocess
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
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


def test_camera_health_core_compiles_and_evaluates(tmp_path: Path):
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

  auto result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "waiting_for_image_publisher");
  assert(!result.stream_healthy);

  snapshot.now_s = 14.0;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "image_publisher_not_present");

  snapshot.image_publisher_present = true;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "stream_metadata_not_received");

  snapshot.stream_metadata_seen = true;
  snapshot.metadata_receipt_time_s = 14.0;
  snapshot.metadata_width = 640;
  snapshot.metadata_height = 480;
  snapshot.metadata_frame_id = "pi_camera_optical_frame";
  snapshot.frames_received = 10;
  snapshot.frame_rate_hz = 29.8;

  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "camera_uncalibrated");
  assert(result.stream_healthy);
  assert(!result.ready_for_pose_estimation);

  const auto text = camera_health_status_text(config, result, snapshot);
  assert(text.find("status=WARN") != std::string::npos);
  assert(text.find("stream_healthy=true") != std::string::npos);
  assert(text.find("image_publisher_present=true") != std::string::npos);
  assert(text.find("stream_metadata_seen=true") != std::string::npos);
  assert(text.find("configured_encoding=rgb8") != std::string::npos);
  assert(text.find("encoding_verification=static_config") != std::string::npos);

  snapshot.camera_calibrated = true;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kOk);
  assert(result.reason == "camera_ready");
  assert(result.ready_for_pose_estimation);

  snapshot.image_publisher_present = false;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "image_publisher_not_present");

  snapshot.image_publisher_present = true;
  snapshot.now_s = 17.0;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "stream_metadata_stale");

  snapshot.now_s = 14.1;
  snapshot.metadata_timestamp_monotonic = false;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "stream_metadata_timestamp_not_monotonic");

  snapshot.metadata_timestamp_monotonic = true;
  snapshot.camera_calibrated = false;
  snapshot.metadata_width = 0;
  snapshot.metadata_height = 0;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "camera_uncalibrated");

  snapshot.camera_calibrated = true;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "stream_metadata_dimensions_invalid");

  snapshot.camera_calibrated = false;
  snapshot.metadata_width = 640;
  snapshot.metadata_height = 0;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kError);
  assert(result.reason == "stream_metadata_dimensions_invalid");

  snapshot.metadata_height = 480;
  snapshot.frame_rate_hz = 4.0;
  result = evaluate_camera_health(config, snapshot);
  assert(result.level == CameraHealthLevel::kWarn);
  assert(result.reason == "low_frame_rate");
  assert(result.stream_healthy);

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
            str(PACKAGE_ROOT / "src/core/camera_health.cpp"),
            "-o",
            str(binary),
        ]
    )
    result = run_command([str(binary)])
    assert "PASS camera_health_core" in result.stdout


def test_camera_health_config_defaults_and_static_encoding_contract():
    health = yaml.safe_load(
        (PACKAGE_ROOT / "config/camera_health.yaml").read_text(encoding="utf-8")
    )["/savo_head/head_camera_status_node"]["ros__parameters"]
    driver = yaml.safe_load(
        (PACKAGE_ROOT / "config/camera_ros.yaml").read_text(encoding="utf-8")
    )["/savo_head/camera_driver"]["ros__parameters"]

    assert health["image_topic"] == "/savo_head/camera/image_raw"
    assert health["camera_info_topic"] == "/savo_head/camera/camera_info"
    assert health["status_topic"] == "/savo_head/camera/status"
    assert health["expected_width"] == 640
    assert health["expected_height"] == 480
    assert health["expected_frame_id"] == "pi_camera_optical_frame"
    assert health["expected_encoding"] == driver["image_encoding"] == "rgb8"
    assert health["metadata_stale_timeout_s"] == 2.0
    assert "image_stale_timeout_s" not in health
    assert "camera_info_stale_timeout_s" not in health
    assert "strict_encoding" not in health
    assert health["require_calibration_for_pose"] is True
