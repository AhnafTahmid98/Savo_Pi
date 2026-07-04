import subprocess
import sys
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def load_params(file_name: str) -> dict:
    path = CONFIG_DIR / file_name
    assert path.is_file(), f"Missing config file: {file_name}"

    data = yaml.safe_load(path.read_text()) or {}
    assert isinstance(data, dict), f"{file_name} root must be a YAML mapping"

    if "savo_head" in data:
        return dict(data.get("savo_head", {}).get("ros__parameters", {}) or {})

    return data


def run_command(cmd: list[str]) -> str:
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

    return result.stdout


def test_head_hardware_defaults():
    params = load_params("head_hardware.yaml")

    assert params["hardware_profile"] == "robot_savo_core_v1"
    assert params["backend"] == "pca9685"
    assert int(params["i2c_bus"]) == 1
    assert int(params["pca9685_address"]) == 0x40

    assert str(params["pan_logical_channel"]) == "7"
    assert str(params["tilt_logical_channel"]) == "6"
    assert int(params["pan_pca9685_channel"]) == 15
    assert int(params["tilt_pca9685_channel"]) == 14

    assert int(params["pan_center_deg"]) == 72
    assert int(params["tilt_center_deg"]) == 55
    assert int(params["tilt_max_deg"]) == 130

    assert not bool(params.get("allow_channel_override", True))


def test_scan_profile_defaults():
    params = load_params("scan_profiles.yaml")

    assert int(params["semantic_scan_pan_min_deg"]) == 0
    assert int(params["semantic_scan_pan_center_deg"]) == 72
    assert int(params["semantic_scan_pan_max_deg"]) == 170

    assert int(params["semantic_scan_tilt_min_deg"]) == 45
    assert int(params["semantic_scan_tilt_max_deg"]) == 130

    assert int(params["semantic_scan_pan_step_deg"]) == 2
    assert int(params["semantic_scan_tilt_step_deg"]) == 2

    assert list(params["semantic_scan_pan_targets_deg"]) == [72, 170, 72, 0]
    assert list(params["semantic_scan_tilt_sweep_pan_targets_deg"]) == [72]


def test_topic_defaults():
    params = load_params("head_topics.yaml")

    expected = {
        "pan_tilt_cmd_topic": "/savo_head/pan_tilt_cmd",
        "pan_tilt_state_topic": "/savo_head/pan_tilt_state",
        "scan_cmd_topic": "/savo_head/scan_cmd",
        "scan_state_topic": "/savo_head/scan_state",
        "status_topic": "/savo_head/status",
        "dashboard_text_topic": "/savo_head/dashboard_text",
        "camera_status_topic": "/savo_head/camera_status",
        "apriltag_detections_topic": "/savo_head/apriltag_detections",
        "semantic_confirmations_topic": "/savo_head/semantic_confirmations",
        "robot_pose_snapshot_topic": "/savo_head/robot_pose_snapshot",
        "diagnostics_topic": "/diagnostics",
    }

    for key, value in expected.items():
        assert params.get(key) == value, f"{key}: expected {value}, got {params.get(key)}"


def test_frame_defaults():
    params = load_params("head_frames.yaml")

    expected = {
        "base_frame": "base_link",
        "pan_frame": "pantilt_pan_link",
        "tilt_frame": "pantilt_tilt_link",
        "camera_frame": "pi_camera_link",
        "camera_optical_frame": "pi_camera_optical_frame",
        "pan_joint_name": "head_pan_joint",
        "tilt_joint_name": "head_tilt_joint",
    }

    for key, value in expected.items():
        assert params.get(key) == value, f"{key}: expected {value}, got {params.get(key)}"

    assert int(params["pan_zero_deg"]) == 72
    assert int(params["tilt_zero_deg"]) == 55


def test_camera_stream_defaults():
    params = load_params("camera_stream.yaml")

    assert params["camera_backend"] == "gstreamer_libcamerasrc"
    assert params["gst_source"] == "libcamerasrc"
    assert params["format"] == "I420"
    assert params["gst_encoder"] == "x264enc"
    assert params["gst_payloader"] == "rtph264pay"
    assert params["gst_sink"] == "udpsink"

    assert int(params["width"]) == 640
    assert int(params["height"]) == 480
    assert int(params["fps"]) == 30
    assert int(params["udp_port"]) == 5000


def test_apriltag_semantic_defaults():
    params = load_params("apriltag_semantics.yaml")

    assert params["apriltag_family"] == "tag36h11"
    assert int(params["min_stable_frames"]) == 5
    assert float(params["min_detection_confidence"]) == 0.70
    assert float(params["max_detection_distance_m"]) == 3.0
    assert float(params["max_detection_age_s"]) == 0.50

    assert bool(params["require_robot_pose"])
    assert bool(params["require_tf_available"])
    assert bool(params["require_robot_stationary"])
    assert bool(params["require_localization_ok"])
    assert bool(params["require_lidar_map_pose"])
    assert bool(params["require_semantic_label"])
    assert not bool(params["allow_unknown_tags"])
    assert bool(params["publish_rejections"])
    assert params["tag_param_prefix"] == "tag_"


def test_diagnostics_defaults():
    text = (CONFIG_DIR / "diagnostics.yaml").read_text().lower()

    for required in [
        "pan",
        "tilt",
        "camera",
        "apriltag",
        "status",
        "stale",
        "timeout",
    ]:
        assert required in text, f"diagnostics defaults missing: {required}"

def test_effective_parameter_dump_matches_defaults():
    output = run_command(
        [
            sys.executable,
            "scripts/dump_effective_head_params.py",
            "--strict",
        ]
    )

    required_output = [
        "Validation    : OK",
        "Errors        : 0",
        "Warnings      : 0",
        "backend          : pca9685",
        "pan              : logical 7 / PCA9685 15",
        "tilt             : logical 6 / PCA9685 14",
        "center           : pan=72°, tilt=55°",
        "tilt max         : 130°",
        "pan targets      : [72, 170, 72, 0]",
        "tilt sweep at    : [72]",
        "backend          : gstreamer_libcamerasrc",
        "source           : libcamerasrc",
        "encoder          : x264enc",
        "payloader        : rtph264pay",
        "sink             : udpsink",
    ]

    missing = [item for item in required_output if item not in output]
    assert not missing, f"dump_effective_head_params.py output missing: {missing}"

    assert "Unknown parameters" not in output


if __name__ == "__main__":
    test_head_hardware_defaults()
    test_scan_profile_defaults()
    test_topic_defaults()
    test_frame_defaults()
    test_camera_stream_defaults()
    test_apriltag_semantic_defaults()
    test_diagnostics_defaults()
    test_effective_parameter_dump_matches_defaults()
    print("PASS: savo_head config defaults match Robot Savo production contracts.")
