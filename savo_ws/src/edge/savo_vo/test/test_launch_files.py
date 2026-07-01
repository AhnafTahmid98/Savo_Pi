from pathlib import Path
import py_compile


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


LAUNCH_FILES = [
    "rgbd_odometry.launch.py",
    "vo_republisher.launch.py",
    "vo_health.launch.py",
    "vo_diagnostics.launch.py",
    "vo_bringup.launch.py",
]


def test_launch_files_exist():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        assert (launch_dir / launch_file).exists(), f"Missing launch file: {launch_file}"


def test_launch_files_compile():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        py_compile.compile(str(launch_dir / launch_file), doraise=True)


def test_launch_files_default_to_cpp():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        text = (launch_dir / launch_file).read_text(encoding="utf-8")
        assert 'default_value="cpp"' in text, f"{launch_file} must default to C++"


def test_launch_files_support_python_fallback():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        text = (launch_dir / launch_file).read_text(encoding="utf-8")
        assert "implementation" in text, f"{launch_file} missing implementation argument"
        assert "_py" in text, f"{launch_file} missing Python fallback support"


def test_launch_files_use_profiles():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        text = (launch_dir / launch_file).read_text(encoding="utf-8")
        assert "profile" in text, f"{launch_file} missing profile argument"
        assert "real_robot_v1" in text, f"{launch_file} missing real_robot_v1 default"
        assert "config" in text, f"{launch_file} missing config path"
        assert "profiles" in text, f"{launch_file} missing profiles path"


def test_bringup_starts_all_vo_nodes():
    text = (PACKAGE_ROOT / "launch" / "vo_bringup.launch.py").read_text(
        encoding="utf-8"
    )

    expected_nodes = [
        "rgbd_odometry_node",
        "vo_republisher_node",
        "vo_health_node",
        "vo_diagnostics_node",
    ]

    for node in expected_nodes:
        assert node in text, f"vo_bringup.launch.py missing node: {node}"


def test_launch_files_do_not_depend_on_camera_package_code():
    launch_dir = PACKAGE_ROOT / "launch"

    for launch_file in LAUNCH_FILES:
        text = (launch_dir / launch_file).read_text(encoding="utf-8")
        assert "savo_realsense" not in text, (
            f"Direct camera package dependency found in {launch_file}"
        )
