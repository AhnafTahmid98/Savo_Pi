import py_compile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch/head_apriltag.launch.py"
LEGACY_CONFIG = ROOT / "config/apriltag_semantics.yaml"


def test_production_apriltag_launch_compiles() -> None:
    py_compile.compile(str(LAUNCH), doraise=True)


def test_production_typed_chain_is_default() -> None:
    text = LAUNCH.read_text(encoding="utf-8")

    required = [
        "head_camera_stack.launch.py",
        '"camera_mode": "ros"',
        "IfCondition",
        'LaunchConfiguration("enable_camera")',
        'executable="apriltag_detector_node"',
        'name="apriltag_detector_node"',
        "apriltag_detector.yaml",
        'executable="apriltag_confirmation_action_node"',
        'name="apriltag_confirmation_action_node"',
        "apriltag_confirmation_action.yaml",
        '"enable_legacy_semantics"',
        'default_value="false"',
        "apriltag_confirm_node",
        "pi_camera_optical_frame",
        "camera_info_url",
    ]
    missing = [item for item in required if item not in text]
    assert not missing, f"production AprilTag launch missing: {missing}"

    obsolete_inline_legacy_parameters = [
        '"require_robot_pose"',
        '"registered_tag_ids_csv"',
        '"tag_param_prefix"',
        '"min_stable_frames"',
        '"min_detection_confidence"',
    ]
    found = [
        item
        for item in obsolete_inline_legacy_parameters
        if item in text
    ]
    assert not found, f"legacy inline launch parameters remain: {found}"


def test_legacy_config_targets_legacy_node_only() -> None:
    text = LEGACY_CONFIG.read_text(encoding="utf-8")
    assert text.startswith("apriltag_confirm_node:\n")
    assert not text.startswith("savo_head:\n")
