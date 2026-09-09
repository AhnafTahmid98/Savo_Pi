from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


LAUNCH_FILES = [
    "rgbd_odometry.launch.py",
    "vo_republisher.launch.py",
    "vo_health.launch.py",
    "vo_diagnostics.launch.py",
    "vo_bringup.launch.py",
]


PROFILE_FILES = [
    "pc_smoke.yaml",
    "real_robot_v1.yaml",
]


CPP_EXECUTABLES = [
    "rgbd_odometry_node",
    "vo_republisher_node",
    "vo_health_node",
    "vo_diagnostics_node",
]


PY_FALLBACK_EXECUTABLES = [
    "rgbd_odometry_node_py",
    "vo_republisher_node_py",
    "vo_health_node_py",
    "vo_diagnostics_node_py",
]


def test_profile_files_exist():
    profile_dir = PACKAGE_ROOT / "config" / "profiles"

    for profile_name in PROFILE_FILES:
        assert (profile_dir / profile_name).exists(), f"Missing profile: {profile_name}"


def test_launch_files_exist_and_compile():
    import py_compile

    launch_dir = PACKAGE_ROOT / "launch"

    for launch_name in LAUNCH_FILES:
        path = launch_dir / launch_name
        assert path.exists(), f"Missing launch file: {launch_name}"
        py_compile.compile(str(path), doraise=True)


def test_launch_files_use_cpp_default_and_py_fallback():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_name in LAUNCH_FILES:
        text = (launch_dir / launch_name).read_text(encoding="utf-8")

        assert 'default_value="cpp"' in text, f"{launch_name} must default to C++"
        assert 'implementation' in text, f"{launch_name} missing implementation argument"
        assert 'profile' in text, f"{launch_name} missing profile argument"
        assert 'real_robot_v1' in text, f"{launch_name} missing real_robot_v1 default profile"

        if launch_name == "vo_bringup.launch.py":
            assert "_select_executable" in text
            assert "_py" in text
        else:
            assert "_py" in text, f"{launch_name} missing Python fallback executable"


def test_cpp_and_python_executable_names_are_present():
    cmake_text = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for executable in CPP_EXECUTABLES:
        assert f"add_executable({executable}" in cmake_text, (
            f"C++ executable not registered in CMakeLists.txt: {executable}"
        )

    for executable in PY_FALLBACK_EXECUTABLES:
        script_path = PACKAGE_ROOT / "scripts" / executable
        assert script_path.exists(), f"Missing Python fallback script: {executable}"
        assert script_path.stat().st_mode & 0o111, (
            f"Python fallback script is not executable: {executable}"
        )
        assert f"scripts/{executable}" in cmake_text, (
            f"Python fallback script not installed by CMake: {executable}"
        )


def test_profiles_wire_raw_and_clean_vo_topics():
    for profile_name in PROFILE_FILES:
        profile_path = PACKAGE_ROOT / "config" / "profiles" / profile_name
        data = yaml.safe_load(profile_path.read_text(encoding="utf-8"))

        rgbd = data["rgbd_odometry_node"]["ros__parameters"]
        republisher = data["vo_republisher_node"]["ros__parameters"]
        health = data["vo_health_node"]["ros__parameters"]
        diagnostics = data["vo_diagnostics_node"]["ros__parameters"]

        assert rgbd["odom_topic"] == "/vo/odom/raw"
        assert rgbd["status_topic"] == "/vo/status"
        assert rgbd["tracking_quality_topic"] == "/vo/tracking_quality"
        assert "health_topic" not in rgbd
        assert rgbd["depth_image_topic"] == (
            "/camera/camera/aligned_depth_to_color/image_raw"
        )
        assert rgbd["base_frame"] == "base_footprint"
        assert rgbd["camera_frame"] == "camera_color_optical_frame"
        assert rgbd["min_depth_correspondences"] >= 4
        assert rgbd["min_pnp_inliers"] >= 4
        assert 0.0 < rgbd["min_pnp_inlier_ratio"] <= 1.0
        assert rgbd["publish_tf"] is False

        assert republisher["odom_raw_topic"] == "/vo/odom/raw"
        assert republisher["odom_topic"] == "/vo/odom"
        assert republisher["base_frame"] == "base_footprint"

        assert health["odom_topic"] == "/vo/odom"
        assert health["health_topic"] == "/vo/health"
        if profile_name == "real_robot_v1.yaml":
            assert health["minimum_rate_hz"] == 5.0
            assert health["good_rate_hz"] == 8.0
            assert health["excellent_rate_hz"] == 12.0

        assert diagnostics["status_topic"] == "/vo/status"
        assert diagnostics["health_topic"] == "/vo/health"
        assert diagnostics["diagnostics_topic"] == "/diagnostics"


def test_vo_health_elapsed_time_uses_steady_clock():
    source = (PACKAGE_ROOT / "src/vo_health_node.cpp").read_text(encoding="utf-8")

    assert "std::chrono::steady_clock::now()" in source
    assert "state_.last_odom_time_s = receipt_time_s;" in source
    assert "state_.last_status_time_s = monotonic_now_s();" in source
    assert "build_health_message(monotonic_now_s())" in source


def test_real_robot_profile_uses_controlled_12_hz_processing():
    profile_path = PACKAGE_ROOT / "config" / "profiles" / "real_robot_v1.yaml"
    data = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    rgbd = data["rgbd_odometry_node"]["ros__parameters"]

    assert rgbd["processing_rate_hz"] == 12.0
    assert rgbd["sync_queue_size"] == 2
    assert rgbd["max_sync_delta_s"] == 0.02
    assert rgbd["max_frame_interval_s"] == 0.20


def test_no_direct_savo_realsense_dependency_in_launch_or_profiles():
    paths = []

    paths.extend((PACKAGE_ROOT / "launch").glob("*.launch.py"))
    paths.extend((PACKAGE_ROOT / "config").glob("*.yaml"))
    paths.extend((PACKAGE_ROOT / "config" / "profiles").glob("*.yaml"))

    for path in paths:
        text = path.read_text(encoding="utf-8")
        assert "savo_realsense" not in text, f"Direct dependency found in {path}"
