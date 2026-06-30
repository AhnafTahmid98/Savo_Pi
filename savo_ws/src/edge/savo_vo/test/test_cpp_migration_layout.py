from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_python_fallback_scripts_exist():
    scripts = PACKAGE_ROOT / "scripts"

    expected = [
        "rgbd_odometry_node_py",
        "vo_health_node_py",
        "vo_diagnostics_node_py",
        "vo_republisher_node_py",
    ]

    for name in expected:
        path = scripts / name
        assert path.exists(), f"Missing fallback script: {name}"
        assert path.stat().st_mode & 0o111, f"Fallback script is not executable: {name}"


def test_no_direct_realsense_code_dependency():
    ignored_dirs = {
        ".git",
        "build",
        "install",
        "log",
        ".pytest_cache",
        "__pycache__",
        "test",
    }

    forbidden_import = "import " + "savo_realsense"
    forbidden_from = "from " + "savo_realsense"

    for path in PACKAGE_ROOT.rglob("*.py"):
        if any(part in ignored_dirs for part in path.relative_to(PACKAGE_ROOT).parts):
            continue

        text = path.read_text(encoding="utf-8")

        assert forbidden_import not in text, f"Direct import found in {path}"
        assert forbidden_from not in text, f"Direct import found in {path}"


def test_constants_importable():
    from savo_vo import constants

    assert constants.PACKAGE_NAME == "savo_vo"
    assert constants.DEFAULT_COLOR_IMAGE_TOPIC == "/camera/camera/color/image_raw"
    assert constants.DEFAULT_DEPTH_IMAGE_TOPIC == "/camera/camera/depth/image_rect_raw"
    assert constants.DEFAULT_VO_ODOM_TOPIC == "/vo/odom"


def test_version_importable():
    from savo_vo.version import get_package_version_info, get_version

    assert get_version() == "0.1.0"
    assert get_package_version_info().package_name == "savo_vo"
