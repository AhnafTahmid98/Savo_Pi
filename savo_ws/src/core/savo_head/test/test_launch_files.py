import py_compile
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


LAUNCH_FILES = [
    "head_bringup.launch.py",
    "head_camera_stream.launch.py",
    "head_camera_view.launch.py",
    "head_debug.launch.py",
    "head_fallback.launch.py",
    "head_hw_only.launch.py",
    "head_scan_test.launch.py",
    "head_apriltag.launch.py",
]


def read_launch(name: str) -> str:
    path = LAUNCH_DIR / name
    assert path.is_file(), f"Missing launch file: {name}"
    return path.read_text()


def assert_contains(text: str, required: list[str], file_name: str) -> None:
    missing = [item for item in required if item not in text]
    assert not missing, f"{file_name} missing: {missing}"


def assert_not_contains(text: str, banned: list[str], file_name: str) -> None:
    found = [item for item in banned if item in text]
    assert not found, f"{file_name} contains banned text: {found}"


def test_all_launch_files_exist():
    for name in LAUNCH_FILES:
        assert (LAUNCH_DIR / name).is_file(), f"Missing launch file: {name}"


def test_all_launch_files_compile():
    for name in LAUNCH_FILES:
        py_compile.compile(str(LAUNCH_DIR / name), doraise=True)


def test_no_unvalidated_camera_commands_in_launch_files():
    banned = [
        "rpicam-hello",
        "rpicam-vid",
        "libcamera-hello",
        "libcamera-vid",
    ]

    for name in LAUNCH_FILES:
        text = read_launch(name)
        assert_not_contains(text, banned, name)


def test_head_bringup_launch_contract():
    name = "head_bringup.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "head_controller_node",
            "head_scan_node",
            "head_tf_node",
            "head_status_node",
            "apriltag_confirm_node",
            "head_camera_stream.launch.py",
            "backend",
            "use_python_fallback",
            "enable_scan",
            "enable_tf",
            "enable_status",
            "enable_apriltag_confirm",
            "enable_camera_stream",
            "camera_source",
            "camera_udp_host",
            "camera_udp_port",
            'default_value="pca9685"',
            'default_value="false"',
            "OpaqueFunction",
            "IncludeLaunchDescription",
            "IfCondition",
            "FindPackageShare",
        ],
        name,
    )


def test_head_camera_stream_launch_contract():
    name = "head_camera_stream.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "gst-launch-1.0",
            "libcamerasrc",
            "videotestsrc",
            "video/x-raw",
            "format={fmt}",
            'default_value="I420"',
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            "rtph264pay",
            "udpsink",
            "udp_host",
            "udp_port",
            "bitrate_kbps",
            'default_value="640"',
            'default_value="480"',
            'default_value="30"',
            'default_value="5000"',
            "OpaqueFunction",
        ],
        name,
    )


def test_head_camera_view_launch_contract():
    name = "head_camera_view.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "gst-launch-1.0",
            "udpsrc",
            "application/x-rtp,media=video,encoding-name=H264,payload=96",
            "rtph264depay",
            "avdec_h264",
            "videoconvert",
            "autovideosink",
            "fbdevsink",
            "fpsdisplaysink",
            "udp_port",
            "show_fps",
            'default_value="5000"',
            "OpaqueFunction",
        ],
        name,
    )


def test_head_debug_launch_contract():
    name = "head_debug.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "head_bringup.launch.py",
            "head_camera_stream.launch.py",
            'default_value="dryrun"',
            'default_value="videotestsrc"',
            "start_camera_test_stream",
            "use_python_fallback",
            "enable_scan",
            "enable_tf",
            "enable_status",
            "enable_apriltag_confirm",
            "center_on_shutdown",
            "udp_host",
            "udp_port",
            "IncludeLaunchDescription",
            "IfCondition",
            "FindPackageShare",
        ],
        name,
    )


def test_head_fallback_launch_contract():
    name = "head_fallback.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "head_bringup.launch.py",
            '"use_python_fallback": "true"',
            'default_value="dryrun"',
            "enable_scan",
            "enable_tf",
            "enable_status",
            "enable_apriltag_confirm",
            "center_on_start",
            "center_on_shutdown",
            "IncludeLaunchDescription",
            "FindPackageShare",
        ],
        name,
    )


def test_head_hw_only_launch_contract():
    name = "head_hw_only.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "head_bringup.launch.py",
            'default_value="pca9685"',
            '"enable_scan": "false"',
            '"enable_tf": "false"',
            '"enable_status": "false"',
            '"enable_apriltag_confirm": "false"',
            "use_python_fallback",
            "center_on_start",
            "center_on_shutdown",
            "IncludeLaunchDescription",
            "FindPackageShare",
        ],
        name,
    )


def test_head_scan_test_launch_contract():
    name = "head_scan_test.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "head_controller_node",
            "head_scan_node",
            'default_value="dryrun"',
            "use_python_fallback",
            "auto_start",
            "center_on_start",
            "center_on_shutdown",
            '"enabled": True',
            "OpaqueFunction",
            "LaunchConfiguration",
        ],
        name,
    )

    assert "head_status_node" not in text
    assert "apriltag_confirm_node" not in text


def test_head_apriltag_launch_contract():
    name = "head_apriltag.launch.py"
    text = read_launch(name)

    assert_contains(
        text,
        [
            "apriltag_confirm_node",
            "head_status_node",
            "head_tf_node",
            "use_python_fallback",
            "tag36h11",
            "min_stable_frames",
            "min_detection_confidence",
            "max_detection_distance_m",
            "max_detection_age_s",
            "require_robot_pose",
            "require_tf_available",
            "require_robot_stationary",
            "require_localization_ok",
            "require_lidar_map_pose",
            "require_semantic_label",
            "allow_unknown_tags",
            "publish_rejections",
            "registered_tag_ids_csv",
            "tag_param_prefix",
            "OpaqueFunction",
        ],
        name,
    )


if __name__ == "__main__":
    test_all_launch_files_exist()
    test_all_launch_files_compile()
    test_no_unvalidated_camera_commands_in_launch_files()
    test_head_bringup_launch_contract()
    test_head_camera_stream_launch_contract()
    test_head_camera_view_launch_contract()
    test_head_debug_launch_contract()
    test_head_fallback_launch_contract()
    test_head_hw_only_launch_contract()
    test_head_scan_test_launch_contract()
    test_head_apriltag_launch_contract()
    print("PASS: savo_head launch files are complete and production-safe.")
