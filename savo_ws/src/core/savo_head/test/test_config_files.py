import subprocess
import sys
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


CONFIG_FILES = [
    "head_hardware.yaml",
    "scan_profiles.yaml",
    "head_topics.yaml",
    "head_frames.yaml",
    "camera_stream.yaml",
    "apriltag_semantics.yaml",
    "diagnostics.yaml",
]


def load_yaml(name: str):
    path = CONFIG_DIR / name
    assert path.is_file(), f"Missing config file: {name}"

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    assert isinstance(data, dict), f"{name} must contain a YAML mapping"
    assert data, f"{name} must not be empty"

    return data


def flatten_values(value):
    if isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from flatten_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from flatten_values(item)
    else:
        yield str(value)


def config_text(name: str) -> str:
    return (CONFIG_DIR / name).read_text(encoding="utf-8")


def assert_any_value_contains(name: str, required: list[str]) -> None:
    data = load_yaml(name)
    values = "\n".join(flatten_values(data))
    missing = [item for item in required if item not in values]
    assert not missing, f"{name} missing expected values/symbols: {missing}"


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


def test_all_config_files_exist_and_parse():
    for name in CONFIG_FILES:
        load_yaml(name)


def test_no_unvalidated_camera_commands_in_config():
    banned = [
        "rpicam-hello",
        "rpicam-vid",
        "libcamera-hello",
        "libcamera-vid",
    ]

    for name in CONFIG_FILES:
        text = config_text(name)
        found = [item for item in banned if item in text]
        assert not found, f"{name} contains banned unvalidated camera command: {found}"


def test_head_hardware_config_locks_pan_tilt_mapping():
    assert_any_value_contains(
        "head_hardware.yaml",
        [
            "pca9685",
            "7",
            "6",
            "15",
            "14",
            "72",
            "55",
            "130",
            "1",
            "64",
        ],
    )


def test_scan_profiles_config_locks_scan_pattern():
    assert_any_value_contains(
        "scan_profiles.yaml",
        [
            "72",
            "170",
            "0",
            "45",
            "130",
            "2",
        ],
    )


def test_head_topics_config_locks_ros_contract():
    assert_any_value_contains(
        "head_topics.yaml",
        [
            "/savo_head/pan_tilt_cmd",
            "/savo_head/pan_tilt_state",
            "/savo_head/scan_cmd",
            "/savo_head/scan_state",
            "/savo_head/status",
            "/savo_head/dashboard_text",
            "/savo_head/apriltag_detections",
            "/savo_head/semantic_confirmations",
            "/savo_head/robot_pose_snapshot",
            "/diagnostics",
        ],
    )


def test_head_frames_config_locks_tf_contract():
    assert_any_value_contains(
        "head_frames.yaml",
        [
            "base_link",
            "pantilt_pan_link",
            "pantilt_tilt_link",
            "pi_camera_link",
            "pi_camera_optical_frame",
            "head_pan_joint",
            "head_tilt_joint",
            "72",
            "55",
        ],
    )


def test_camera_stream_config_locks_validated_gstreamer_path():
    assert_any_value_contains(
        "camera_stream.yaml",
        [
            "gstreamer_libcamerasrc",
            "libcamerasrc",
            "I420",
            "x264enc",
            "rtph264pay",
            "udpsink",
            "640",
            "480",
            "30",
            "5000",
        ],
    )


def test_apriltag_semantics_config_locks_confirmation_policy():
    assert_any_value_contains(
        "apriltag_semantics.yaml",
        [
            "tag36h11",
            "5",
            "0.7",
            "3.0",
            "0.5",
        ],
    )

    text = config_text("apriltag_semantics.yaml")
    for symbol in [
        "require_robot_pose",
        "require_tf_available",
        "require_robot_stationary",
        "require_localization_ok",
        "require_lidar_map_pose",
        "require_semantic_label",
        "allow_unknown_tags",
        "publish_rejections",
        "tag_param_prefix",
    ]:
        assert symbol in text, f"apriltag_semantics.yaml missing: {symbol}"


def test_diagnostics_config_locks_status_policy():
    text = config_text("diagnostics.yaml")

    for symbol in [
        "status",
        "stale",
        "timeout",
        "pan",
        "tilt",
        "camera",
        "apriltag",
    ]:
        assert symbol in text.lower(), f"diagnostics.yaml missing diagnostic concept: {symbol}"


def test_dump_effective_head_params_strict_passes():
    script = PACKAGE_ROOT / "scripts" / "dump_effective_head_params.py"
    assert script.is_file(), "Missing scripts/dump_effective_head_params.py"

    run_command(
        [
            sys.executable,
            str(script),
            "--strict",
        ]
    )


if __name__ == "__main__":
    test_all_config_files_exist_and_parse()
    test_no_unvalidated_camera_commands_in_config()
    test_head_hardware_config_locks_pan_tilt_mapping()
    test_scan_profiles_config_locks_scan_pattern()
    test_head_topics_config_locks_ros_contract()
    test_head_frames_config_locks_tf_contract()
    test_camera_stream_config_locks_validated_gstreamer_path()
    test_apriltag_semantics_config_locks_confirmation_policy()
    test_diagnostics_config_locks_status_policy()
    test_dump_effective_head_params_strict_passes()
    print("PASS: savo_head config files parse and lock hardware, topics, frames, camera, scan, AprilTag, and diagnostics.")
