import math
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_head_tf_config_is_production_calibrated_and_measured() -> None:
    config_path = ROOT / "config/head_frames.yaml"
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert set(data) == {"head_tf_node"}
    params = data["head_tf_node"]["ros__parameters"]

    assert params["publish_tf"] is True
    assert params["transforms_calibrated"] is True
    assert params["require_valid_pan_tilt_state"] is True
    assert params["stale_state_timeout_s"] == pytest.approx(0.50)
    assert params["base_frame"] == "pantilt_mount_link"
    assert params["pan_frame"] == "pantilt_pan_link"
    assert params["tilt_frame"] == "pantilt_tilt_link"
    assert params["camera_frame"] == "pi_camera_link"
    assert params["camera_optical_frame"] == "pi_camera_optical_frame"
    assert params["pan_axis"] == "z"
    assert params["tilt_axis"] == "y"
    assert params["pan_sign"] == pytest.approx(1.0)
    assert params["tilt_sign"] == pytest.approx(-1.0)
    assert params["pan_zero_deg"] == pytest.approx(72.0)
    assert params["tilt_zero_deg"] == pytest.approx(55.0)
    assert params["base_to_pan_xyz_m"] == [0.0, 0.0, 0.0]
    assert params["base_to_pan_rpy_rad"] == [0.0, 0.0, 0.0]
    assert params["pan_to_tilt_xyz_m"] == [0.0, 0.0, 0.046]
    assert params["pan_to_tilt_rpy_rad"] == [0.0, 0.0, 0.0]
    assert params["tilt_to_camera_xyz_m"] == [0.025, 0.0, -0.010]
    assert params["tilt_to_camera_rpy_rad"] == [0.0, 0.0, 0.0]
    assert params["camera_to_optical_xyz_m"] == [0.0, 0.0, 0.0]
    assert params["camera_to_optical_rpy_rad"] == [
        -1.57079632679,
        0.0,
        -1.57079632679,
    ]

    pan_relative = params["pan_sign"] * (
        math.radians(72.0) - math.radians(params["pan_zero_deg"])
    )
    tilt_relative = params["tilt_sign"] * (
        math.radians(55.0) - math.radians(params["tilt_zero_deg"])
    )
    assert pan_relative == pytest.approx(0.0, abs=1e-15)
    assert tilt_relative == pytest.approx(0.0, abs=1e-15)


def test_cpp_and_python_tf_nodes_retain_fail_closed_runtime_gates() -> None:
    cpp = (ROOT / "src/nodes/head_tf_node.cpp").read_text(encoding="utf-8")
    python = (
        ROOT / "savo_head/nodes/head_tf_node.py"
    ).read_text(encoding="utf-8")

    for source in (cpp, python):
        assert "transforms_calibrated" in source
        assert "physical_head_transforms_not_calibrated" in source
        assert "publish_tf" in source

    assert 'declare_parameter<bool>("publish_tf", false)' in cpp
    assert 'declare_parameter("publish_tf", False)' in python

    assert cpp.index('get_parameter("transforms_calibrated")') < cpp.index(
        "tf_broadcaster_->sendTransform"
    )
    assert python.index('get_parameter("transforms_calibrated")') < python.index(
        "self._tf_broadcaster.sendTransform"
    )

    cpp_angles = cpp.split("std::pair<double, double> current_joint_angles()", 1)[1]
    cpp_angles = cpp_angles.split("static double joint_position", 1)[0]
    python_angles = python.split("def _current_joint_angles", 1)[1]
    python_angles = python_angles.split("def _joint_position", 1)[0]
    for source in (cpp_angles, python_angles):
        assert "require_valid_pan_tilt_state" in source
        assert "no pan-tilt joint state received" in source
        assert "stale_state_timeout_s" in source
        assert "pan-tilt joint state is stale" in source


def test_head_launches_apply_config_without_duplicate_tf_publishers() -> None:
    for name in ("head_bringup.launch.py", "head_apriltag.launch.py"):
        text = (ROOT / f"launch/{name}").read_text(encoding="utf-8")
        assert "head_frames_config_file" in text
        assert "head_frames.yaml" in text
        assert 'name="head_tf_node"' in text
        assert 'LaunchConfiguration("head_frames_config_file")' in text
        assert text.count('name="head_tf_node"') == 1
        assert "static_transform_publisher" not in text
        assert "StaticTransformBroadcaster" not in text

    bringup = (ROOT / "launch/head_bringup.launch.py").read_text(encoding="utf-8")
    apriltag = (ROOT / "launch/head_apriltag.launch.py").read_text(encoding="utf-8")
    assert '"enable_tf",\n                default_value="true"' in bringup
    assert '"enable_tf",\n                default_value="false"' in apriltag


def test_savo_head_does_not_claim_description_owned_mount_transform() -> None:
    config = (ROOT / "config/head_frames.yaml").read_text(encoding="utf-8")
    cpp = (ROOT / "src/nodes/head_tf_node.cpp").read_text(encoding="utf-8")
    python = (ROOT / "savo_head/nodes/head_tf_node.py").read_text(encoding="utf-8")

    assert 'base_frame: "pantilt_mount_link"' in config
    for source in (cpp, python):
        assert "StaticTransformBroadcaster" not in source
        assert "static_transform_publisher" not in source


def test_controller_joint_state_remains_radians_for_tf_consumers() -> None:
    cpp = (ROOT / "src/nodes/head_controller_node.cpp").read_text(encoding="utf-8")
    python = (
        ROOT / "savo_head/nodes/head_controller_node.py"
    ).read_text(encoding="utf-8")
    tf_cpp = (ROOT / "src/nodes/head_tf_node.cpp").read_text(encoding="utf-8")
    tf_python = (
        ROOT / "savo_head/nodes/head_tf_node.py"
    ).read_text(encoding="utf-8")

    assert "degrees_to_radians(state.pan_deg)" in cpp
    assert "degrees_to_radians(state.tilt_deg)" in cpp
    assert "math.radians(float(state.pan_deg))" in python
    assert "math.radians(float(state.tilt_deg))" in python
    assert "pan_abs - pan_zero" in tf_cpp
    assert "tilt_abs - tilt_zero" in tf_cpp
    assert "pan_abs - pan_zero" in tf_python
    assert "tilt_abs - tilt_zero" in tf_python


def test_production_apriltag_path_has_no_robot_pose_snapshot_dependency() -> None:
    production_files = [
        ROOT / "launch/head_apriltag.launch.py",
        ROOT / "config/apriltag_detector.yaml",
        ROOT / "config/apriltag_confirmation_action.yaml",
        ROOT / "src/nodes/apriltag_detector_node.cpp",
        ROOT / "src/nodes/apriltag_confirmation_action_node.cpp",
    ]

    for path in production_files:
        text = path.read_text(encoding="utf-8")
        assert "/savo_head/robot_pose_snapshot" not in text, path
