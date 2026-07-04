import subprocess
import tempfile
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def run_command(cmd: list[str]) -> None:
    result = subprocess.run(
        cmd,
        cwd=PACKAGE_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, (
        "Command failed:\n"
        + " ".join(cmd)
        + "\n\nSTDOUT:\n"
        + result.stdout
        + "\nSTDERR:\n"
        + result.stderr
    )


def compile_and_run(name: str, code: str, sources: list[str] | None = None) -> None:
    sources = sources or []

    with tempfile.TemporaryDirectory(prefix=f"savo_head_{name}_") as tmp:
        tmp_path = Path(tmp)
        cpp_path = tmp_path / f"{name}.cpp"
        binary_path = tmp_path / name

        cpp_path.write_text(code)

        cmd = [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-I",
            str(PACKAGE_ROOT / "include"),
            str(cpp_path),
            *[str(PACKAGE_ROOT / source) for source in sources],
            "-o",
            str(binary_path),
        ]

        run_command(cmd)
        run_command([str(binary_path)])


def read_file(relative_path: str) -> str:
    path = PACKAGE_ROOT / relative_path
    assert path.is_file(), f"Missing file: {relative_path}"
    return path.read_text()


def assert_contains(relative_path: str, required: list[str]) -> None:
    text = read_file(relative_path)
    missing = [item for item in required if item not in text]
    assert not missing, f"{relative_path} missing: {missing}"


def test_cpp_core_header_files_exist():
    for relative_path in [
        "include/savo_head/core/head_types.hpp",
        "include/savo_head/core/servo_calibration.hpp",
        "include/savo_head/core/scan_pattern.hpp",
        "include/savo_head/core/diagnostics.hpp",
        "include/savo_head/core/topic_contract.hpp",
        "include/savo_head/core/frame_contract.hpp",
        "include/savo_head/core/head_state.hpp",
        "include/savo_head/core/pantilt_command.hpp",
        "include/savo_head/core/head_config.hpp",
        "include/savo_head/core/head_status.hpp",
    ]:
        assert (PACKAGE_ROOT / relative_path).is_file(), f"Missing file: {relative_path}"


def test_cpp_driver_header_files_exist():
    for relative_path in [
        "include/savo_head/drivers/pca9685_driver.hpp",
        "include/savo_head/drivers/pantilt_driver.hpp",
    ]:
        assert (PACKAGE_ROOT / relative_path).is_file(), f"Missing file: {relative_path}"


def test_cpp_source_files_exist():
    for relative_path in [
        "src/core/head_state.cpp",
        "src/core/scan_pattern.cpp",
        "src/core/servo_calibration.cpp",
        "src/core/head_config.cpp",
        "src/core/head_status.cpp",
        "src/drivers/pca9685_driver.cpp",
        "src/drivers/pantilt_driver.cpp",
    ]:
        assert (PACKAGE_ROOT / relative_path).is_file(), f"Missing file: {relative_path}"


def test_head_types_contract_static_content():
    assert_contains(
        "include/savo_head/core/head_types.hpp",
        [
            "kPanCenterDeg = 72",
            "kTiltCenterDeg = 55",
            "kPanMinDeg = 0",
            "kPanMaxDeg = 170",
            "kTiltMinDeg = 45",
            "kTiltMaxDeg = 130",
            "kManualStepDeg = 2",
            "enum class HeadMode",
            "enum class HeadStatus",
            "enum class CommandType",
            "enum class CommandSource",
            "struct PanTiltLimits",
            "struct PanTiltState",
            "struct PanTiltCommand",
        ],
    )


def test_servo_calibration_contract_static_content():
    assert_contains(
        "include/savo_head/core/servo_calibration.hpp",
        [
            "kI2cBusDefault",
            "kPca9685AddressDefault",
            "kPca9685PwmFrequencyHzDefault",
            "kPanLogicalChannel",
            "kTiltLogicalChannel",
            "kPanPca9685Channel",
            "kTiltPca9685Channel",
            "logical_channel_to_pca9685_channel",
            "angle_to_servo_output",
        ],
    )


def test_topic_contract_static_content():
    assert_contains(
        "include/savo_head/core/topic_contract.hpp",
        [
            "HeadTopicContract",
            "HeadQosContract",
            "pan_tilt_cmd",
            "pan_tilt_state",
            "scan_cmd",
            "scan_state",
            "status",
            "dashboard_text",
            "apriltag_detections",
            "semantic_confirmations",
            "robot_pose_snapshot",
            "diagnostics",
            "best_effort",
            "reliable",
            "should_use_best_effort_for_topic",
        ],
    )

def test_frame_contract_static_content():
    assert_contains(
        "include/savo_head/core/frame_contract.hpp",
        [
            "HeadFrameContract",
            "base_frame",
            "pan_frame",
            "tilt_frame",
            "camera_frame",
            "camera_optical_frame",
            "pan_joint",
            "tilt_joint",
            "pan_zero_deg",
            "tilt_zero_deg",
            "pan_joint_offset_rad",
            "tilt_joint_offset_rad",
            "default_head_frame_contract",
        ],
    )

def test_pantilt_command_contract_static_content():
    assert_contains(
        "include/savo_head/core/pantilt_command.hpp",
        [
            "kVectorCmdAbsolute = 0.0",
            "kVectorCmdDelta = 1.0",
            "kVectorCmdCenter = 2.0",
            "kVectorCmdHold = 3.0",
            "kVectorCmdStop = 4.0",
            "manual_key_command",
            "command_from_vector",
            "command_to_vector",
            "command_contract_valid",
            "command_is_safety_action",
        ],
    )


def test_head_config_static_content():
    assert_contains(
        "include/savo_head/core/head_config.hpp",
        [
            "robot_savo_core_v1",
            "gstreamer_libcamerasrc",
            "opencv_videocapture_unvalidated",
            "HeadHardwareConfig",
            "HeadCameraConfig",
            "HeadAprilTagConfig",
            "HeadDiagnosticsConfig",
            "HeadConfig",
            "default_head_config",
        ],
    )


def test_compile_head_types_contract():
    compile_and_run(
        "head_types_contract",
        r'''
#include <cassert>
#include <iostream>
#include "savo_head/core/head_types.hpp"

int main()
{
  using namespace savo_head;

  static_assert(kPanCenterDeg == 72);
  static_assert(kTiltCenterDeg == 55);
  static_assert(kTiltMaxDeg == 130);
  static_assert(kManualStepDeg == 2);

  PanTiltLimits limits;
  assert(limits.clamp_pan(-10) == 0);
  assert(limits.clamp_pan(999) == 170);
  assert(limits.clamp_tilt(-10) == 45);
  assert(limits.clamp_tilt(999) == 130);

  auto state = PanTiltState{999, -10, HeadMode::kManual, HeadStatus::kOk, 1.0, "unit"};
  state = state.normalized(limits);
  assert(state.pan_deg == 170);
  assert(state.tilt_deg == 45);

  auto center = center_command(2.0, CommandSource::kService);
  assert(center.type == CommandType::kCenter);
  assert(center.source == CommandSource::kService);

  std::cout << "PASS head_types_contract\n";
  return 0;
}
''',
    )


def test_compile_servo_calibration_contract():
    compile_and_run(
        "servo_calibration_contract",
        r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include "savo_head/core/servo_calibration.hpp"

int main()
{
  using namespace savo_head;

  const auto calibration = default_servo_calibration();
  assert(calibration.valid());

  assert(logical_channel_to_pca9685_channel("7") == 15);
  assert(logical_channel_to_pca9685_channel("6") == 14);

  const auto pan = angle_to_servo_output("pan", 72, calibration);
  assert(pan.pca9685_channel == 15);
  assert(pan.angle_deg == 72);
  assert(std::abs(pan.pulse_us - 1411.1111111111113) < 1e-9);
  assert(pan.ticks == 288);

  const auto tilt = angle_to_servo_output("tilt", 55, calibration);
  assert(tilt.pca9685_channel == 14);
  assert(tilt.angle_deg == 55);
  assert(std::abs(tilt.pulse_us - 1222.2222222222222) < 1e-9);
  assert(tilt.ticks == 250);

  std::cout << "PASS servo_calibration_contract\n";
  return 0;
}
''',
        ["src/core/servo_calibration.cpp"],
    )


def test_compile_scan_pattern_contract():
    compile_and_run(
        "scan_pattern_contract",
        r'''
#include <cassert>
#include <iostream>
#include <vector>
#include "savo_head/core/scan_pattern.hpp"

int main()
{
  using namespace savo_head;

  auto profile = ScanProfile{}.normalized();
  assert(profile.valid());
  assert(profile.pan_targets_deg == std::vector<int>({72, 170, 72, 0}));
  assert(profile.tilt_sweep_pan_targets_deg == std::vector<int>({72}));
  assert(profile.pan_step_deg == 2);
  assert(profile.tilt_step_deg == 2);
  assert(profile.tilt_min_deg == 45);
  assert(profile.tilt_max_deg == 130);

  auto runtime = make_scan_runtime(profile).start(1.0);
  assert(runtime.running());
  assert(runtime.status.pan_deg == 0);
  assert(runtime.status.tilt_deg == 45);
  assert(runtime.status.current_pan_target_deg == 72);

  std::cout << "PASS scan_pattern_contract\n";
  return 0;
}
''',
        ["src/core/scan_pattern.cpp"],
    )


def test_compile_topic_and_frame_contracts():
    compile_and_run(
        "topic_frame_contracts",
        r'''
#include <cassert>
#include <iostream>
#include <string>
#include "savo_head/core/topic_contract.hpp"
#include "savo_head/core/frame_contract.hpp"

int main()
{
  using namespace savo_head;

  const auto topics = default_head_topic_contract();
  assert(topics.valid());
  assert(topics.pan_tilt_cmd == "/savo_head/pan_tilt_cmd");
  assert(topics.semantic_confirmations == "/savo_head/semantic_confirmations");

  const auto qos = default_head_qos_contract();
  assert(qos.valid());
  assert(qos.sensor_reliability == "best_effort");
  assert(qos.command_reliability == "reliable");
  assert(should_use_best_effort_for_topic(topics.apriltag_detections));
  assert(!should_use_best_effort_for_topic(topics.semantic_confirmations));

  const auto frames = default_head_frame_contract();
  assert(frames.valid());
  assert(frames.base_frame == "base_link");
  assert(frames.pan_frame == "pantilt_pan_link");
  assert(frames.tilt_frame == "pantilt_tilt_link");
  assert(frames.camera_frame == "pi_camera_link");
  assert(frames.camera_optical_frame == "pi_camera_optical_frame");
  assert(frames.pan_zero_deg == 72.0);
  assert(frames.tilt_zero_deg == 55.0);

  std::cout << "PASS topic_frame_contracts\n";
  return 0;
}
''',
    )


def test_compile_pantilt_command_contract():
    compile_and_run(
        "pantilt_command_contract",
        r'''
#include <cassert>
#include <iostream>
#include "savo_head/core/pantilt_command.hpp"

int main()
{
  using namespace savo_head;

  auto vector = command_to_vector(absolute_command(72, 55, 1.0));
  assert(vector.x == 72.0);
  assert(vector.y == 55.0);
  assert(vector.z == kVectorCmdAbsolute);

  auto command = command_from_vector(VectorPanTiltCommand{2.0, -2.0, kVectorCmdDelta}, 2.0);
  assert(command.type == CommandType::kDelta);
  assert(command.pan_delta_deg == 2);
  assert(command.tilt_delta_deg == -2);

  auto hold = manual_key_command(" ", 2, 3.0);
  assert(hold.has_value());
  assert(hold->type == CommandType::kHold);

  auto right = manual_key_command("d", 2, 3.0);
  assert(right.has_value());
  assert(right->pan_delta_deg == 2);

  assert(command_contract_valid(absolute_command(72, 55, 1.0)));
  assert(!command_contract_valid(absolute_command(999, 999, 1.0)));

  std::cout << "PASS pantilt_command_contract\n";
  return 0;
}
''',
    )


def test_compile_head_state_contract():
    compile_and_run(
        "head_state_contract",
        r'''
#include <cassert>
#include <iostream>
#include <string>
#include "savo_head/core/head_state.hpp"

int main()
{
  using namespace savo_head;

  PanTiltLimits limits;
  auto runtime = make_initial_head_state(limits, HeadStatus::kDryrun, true, 1.0);
  assert(runtime.pan_tilt.pan_deg == 72);
  assert(runtime.pan_tilt.tilt_deg == 55);
  assert(runtime.dryrun);

  PanTiltState current;
  current.pan_deg = 72;
  current.tilt_deg = 55;
  current.mode = HeadMode::kIdle;
  current.status = HeadStatus::kOk;
  current.stamp_s = 1.0;

  auto transition = apply_head_command_to_state(
    current,
    delta_command(2, 2, 2.0, CommandSource::kTopic),
    limits);

  assert(transition.ok());
  assert(transition.current.pan_deg == 74);
  assert(transition.current.tilt_deg == 57);
  assert(transition.current.mode == HeadMode::kManual);

  transition = apply_head_command_to_state(
    transition.current,
    center_command(3.0, CommandSource::kService),
    limits);

  assert(transition.ok());
  assert(transition.current.pan_deg == 72);
  assert(transition.current.tilt_deg == 55);
  assert(transition.current.mode == HeadMode::kCentering);

  assert(head_mode_from_string("manual").value() == HeadMode::kManual);
  assert(head_status_from_string("dryrun").value() == HeadStatus::kDryrun);

  std::cout << "PASS head_state_contract\n";
  return 0;
}
''',
        ["src/core/head_state.cpp"],
    )


def test_compile_head_config_contract():
    compile_and_run(
        "head_config_contract",
        r'''
#include <cassert>
#include <iostream>
#include <vector>
#include "savo_head/core/head_config.hpp"

int main()
{
  using namespace savo_head;

  const auto config = default_head_config();
  assert(config.valid());

  assert(config.hardware.backend == "pca9685");
  assert(config.hardware.i2c_bus == 1);
  assert(config.hardware.pca9685_address == 0x40);
  assert(config.hardware.calibration.pan.logical_channel == "7");
  assert(config.hardware.calibration.pan.pca9685_channel == 15);
  assert(config.hardware.calibration.tilt.logical_channel == "6");
  assert(config.hardware.calibration.tilt.pca9685_channel == 14);

  assert(config.scan.pan_targets_deg == std::vector<int>({72, 170, 72, 0}));
  assert(config.camera.camera_backend == "gstreamer_libcamerasrc");
  assert(config.camera.gst_source == "libcamerasrc");
  assert(config.camera.gst_encoder == "x264enc");
  assert(config.camera.gst_payloader == "rtph264pay");
  assert(config.camera.gst_sink == "udpsink");

  assert(config.apriltag.enabled);
  assert(config.apriltag.family == "tag36h11");
  assert(config.apriltag.require_robot_pose);

  std::cout << "PASS head_config_contract\n";
  return 0;
}
''',
        [
            "src/core/head_config.cpp",
            "src/core/scan_pattern.cpp",
            "src/core/servo_calibration.cpp",
        ],
    )


def test_compile_head_status_contract():
    compile_and_run(
        "head_status_contract",
        r'''
#include <cassert>
#include <iostream>
#include <string>
#include "savo_head/core/head_status.hpp"

int main()
{
  using namespace savo_head;

  HeadStatusPolicy policy;
  policy.require_controller = true;
  policy.require_camera = false;

  auto snapshot = make_status_snapshot_from_policy(policy, 10.0);
  assert(snapshot.controller.required);
  assert(!snapshot.camera.required);

  snapshot.controller.update(10.0, "controller_ok");
  assert(snapshot.controller.status(10.1) == HeadStatus::kOk);
  assert(snapshot.camera.status(10.1) == HeadStatus::kDryrun);

  const auto dashboard = head_status_dashboard_text(snapshot, 10.1, true);
  assert(dashboard.find("savo_head status=OK_WITH_WARNINGS") != std::string::npos);

  HeadRuntimeState runtime;
  runtime.pan_tilt = make_center_state(20.0, HeadStatus::kOk, "unit");
  runtime.driver_opened = true;
  runtime.dryrun = true;

  auto health = runtime_state_health(runtime, 20.1, 1.0);
  assert(health.name == "savo_head.runtime");
  assert(health.level == DiagnosticLevel::kOk);

  std::cout << "PASS head_status_contract\n";
  return 0;
}
''',
        ["src/core/head_state.cpp", "src/core/head_status.cpp"],
    )


def test_compile_driver_dryrun_contract():
    compile_and_run(
        "driver_dryrun_contract",
        r"""
#include <cassert>
#include <iostream>
#include "savo_head/drivers/pantilt_driver.hpp"

int main()
{
  using namespace savo_head;

  auto config = dryrun_pantilt_config();
  config = config.normalized();

  assert(config.backend == "dryrun");
  assert(config.calibration.valid());

  PanTiltDriver driver(config);

  driver.open();

  auto state = driver.center(CommandSource::kSystem, 1.0);
  assert(state.pan_deg == 72);
  assert(state.tilt_deg == 55);

  state = driver.apply_command(delta_command(2, 2, 2.0, CommandSource::kManual));
  assert(state.pan_deg == 74);
  assert(state.tilt_deg == 57);

  state = driver.apply_command(center_command(3.0, CommandSource::kService));
  assert(state.pan_deg == 72);
  assert(state.tilt_deg == 55);

  state = driver.hold(CommandSource::kSystem, 4.0);
  assert(state.pan_deg == 72);
  assert(state.tilt_deg == 55);

  driver.close();

  std::cout << "PASS driver_dryrun_contract\n";
  return 0;
}
""",
        [
            "src/core/head_state.cpp",
            "src/drivers/pca9685_driver.cpp",
            "src/drivers/pantilt_driver.cpp",
        ],
    )


if __name__ == "__main__":
    test_cpp_core_header_files_exist()
    test_cpp_driver_header_files_exist()
    test_cpp_source_files_exist()
    test_head_types_contract_static_content()
    test_servo_calibration_contract_static_content()
    test_topic_contract_static_content()
    test_frame_contract_static_content()
    test_pantilt_command_contract_static_content()
    test_head_config_static_content()
    test_compile_head_types_contract()
    test_compile_servo_calibration_contract()
    test_compile_scan_pattern_contract()
    test_compile_topic_and_frame_contracts()
    test_compile_pantilt_command_contract()
    test_compile_head_state_contract()
    test_compile_head_config_contract()
    test_compile_head_status_contract()
    test_compile_driver_dryrun_contract()
    print("PASS: savo_head C++ contracts compile and validate without ROS runtime.")
