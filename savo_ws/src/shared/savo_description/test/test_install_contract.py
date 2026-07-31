"""Validate that description assets are installed by ament CMake."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_description_assets_are_installed() -> None:
    """Require every runtime asset directory in the install rule."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "install(" in cmake
    assert "DIRECTORY" in cmake

    for asset_dir in ("launch", "urdf", "config", "rviz", "meshes", "scripts"):
        assert asset_dir in cmake

    assert "DESTINATION share/${PROJECT_NAME}" in cmake
    assert "README.md" in cmake
    assert "package.xml" in cmake


def test_description_tests_are_registered() -> None:
    """Keep source and geometry contract tests in the CTest graph."""
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    for test_name in (
        "test_urdf_exists",
        "test_xacro_builds",
        "test_required_frames",
        "test_config_files",
        "test_no_duplicate_frames",
        "test_install_contract",
        "test_geometry_contract",
    ):
        assert f"ament_add_pytest_test({test_name}" in cmake
