import subprocess
from pathlib import Path


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
        'Command failed:\n'
        + ' '.join(command)
        + '\n\nSTDOUT:\n'
        + result.stdout
        + '\nSTDERR:\n'
        + result.stderr
    )

    return result


def test_camera_status_report_core(
    tmp_path: Path,
):
    source = tmp_path / 'camera_status_report.cpp'
    binary = tmp_path / 'camera_status_report'

    source.write_text(
        r'''
#include <cassert>
#include <iostream>
#include <string>

#include "savo_head/core/camera_status_report.hpp"

int main()
{
  using namespace savo_head;

  const auto warning =
    parse_camera_status_text(
      "status=WARN "
      "reason=camera_uncalibrated "
      "stream_healthy=true "
      "ready_for_pose_estimation=false "
      "calibrated=false");

  assert(warning.valid);
  assert(
    warning.severity ==
    CameraReportedSeverity::kWarn);

  assert(warning.status == "WARN");
  assert(
    warning.reason ==
    "camera_uncalibrated");

  assert(
    warning.bool_value(
      "stream_healthy").value());

  assert(
    !warning.bool_value(
      "ready_for_pose_estimation").value());

  CameraStatusContext context;
  context.seen = true;
  context.required = true;
  context.age_s = 0.1;
  context.stale_timeout_s = 2.0;
  context.raw_text =
    "status=WARN "
    "reason=camera_uncalibrated "
    "stream_healthy=true "
    "ready_for_pose_estimation=false";

  auto health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kWarn);

  assert(
    health.status ==
    HeadStatus::kOk);

  assert(
    health.message ==
    "camera_uncalibrated");

  context.raw_text =
    "status=OK "
    "reason=camera_ready "
    "stream_healthy=true "
    "ready_for_pose_estimation=true";

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kOk);

  assert(health.ok());

  context.raw_text =
    "status=ERROR "
    "reason=image_stale "
    "stream_healthy=false "
    "ready_for_pose_estimation=false";

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kError);

  assert(
    health.status ==
    HeadStatus::kError);

  assert(health.message == "image_stale");

  context.seen = false;
  context.required = false;
  context.raw_text.clear();

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kWarn);

  assert(
    health.message ==
    "camera_status_missing");

  context.required = true;

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kError);

  context.seen = true;
  context.required = false;
  context.age_s = 3.0;
  context.stale_timeout_s = 2.0;

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kWarn);

  assert(
    health.message ==
    "camera_status_stale");

  context.required = true;

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kError);

  context.age_s = 0.1;
  context.raw_text =
    "this-is-not-a-valid-camera-status";

  health = camera_status_health(context);

  assert(
    health.level ==
    DiagnosticLevel::kError);

  assert(
    health.message ==
    "camera_status_invalid");

  std::cout
    << "PASS camera_status_report_core\n";

  return 0;
}
''',
        encoding='utf-8',
    )

    run_command(
        [
            'g++',
            '-std=c++17',
            '-Wall',
            '-Wextra',
            '-Wpedantic',
            '-I',
            str(PACKAGE_ROOT / 'include'),
            str(source),
            str(
                PACKAGE_ROOT
                / 'src/core/camera_status_report.cpp'
            ),
            '-o',
            str(binary),
        ]
    )

    result = run_command([str(binary)])

    assert (
        'PASS camera_status_report_core'
        in result.stdout
    )
