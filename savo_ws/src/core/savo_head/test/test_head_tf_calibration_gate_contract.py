from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_head_tf_config_has_measured_translations_but_stays_disabled() -> None:
    config_path = ROOT / "config/head_frames.yaml"
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert set(data) == {"head_tf_node"}
    params = data["head_tf_node"]["ros__parameters"]

    assert params["publish_tf"] is False
    assert params["transforms_calibrated"] is False
    assert params["base_to_pan_xyz_m"] == [0.0, 0.0, 0.0]
    assert params["pan_to_tilt_xyz_m"] == [0.0, 0.0, 0.046]
    assert params["tilt_to_camera_xyz_m"] == [0.025, 0.0, -0.010]
    assert params["camera_to_optical_rpy_rad"] == [
        -1.57079632679,
        0.0,
        -1.57079632679,
    ]


def test_cpp_and_python_tf_nodes_refuse_uncalibrated_publication() -> None:
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


def test_head_launches_apply_the_calibration_gated_tf_config() -> None:
    for name in ("head_bringup.launch.py", "head_apriltag.launch.py"):
        text = (ROOT / f"launch/{name}").read_text(encoding="utf-8")
        assert "head_frames_config_file" in text
        assert "head_frames.yaml" in text
        assert 'name="head_tf_node"' in text
        assert 'LaunchConfiguration("head_frames_config_file")' in text


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
