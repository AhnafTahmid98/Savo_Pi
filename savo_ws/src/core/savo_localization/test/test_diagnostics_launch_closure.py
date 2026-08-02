from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launches_do_not_reference_removed_empty_compatibility_nodes() -> None:
    forbidden = {
        "localization_dashboard.py",
        "ekf_state_publisher_node.py",
        "wheel_odom_fallback_node.py",
    }
    for launch_file in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
        content = launch_file.read_text(encoding="utf-8")
        assert not (forbidden & set(content.split('"'))), launch_file


def test_diagnostics_uses_cpp_health_and_observer_owns_visualization() -> None:
    diagnostics = (PACKAGE_ROOT / "launch/diagnostics.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="localization_health_node"' in diagnostics
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    assert "savo_observer" in readme
    assert "wheel odometry fallback" in readme.lower()


def test_production_localization_authorities_unchanged() -> None:
    bringup = (PACKAGE_ROOT / "launch/localization_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="imu_node"' in bringup
    assert 'executable="wheel_odom_node"' in bringup
    assert 'executable="ekf_node"' in bringup
    assert 'executable="localization_health_node"' in bringup
    assert "fallback" not in bringup.lower()
