"""Check observer launch composition and argument contracts."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch'


def test_all_launch_files_compile_and_expected_entry_points_exist():
    expected = {
        'observer.launch.py', 'rviz_observer.launch.py',
        'dashboard_observer.launch.py', 'full_observer.launch.py',
        'observer_bridge.launch.py',
    }
    assert {path.name for path in LAUNCH.glob('*.launch.py')} == expected
    for path in LAUNCH.glob('*.launch.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))


def test_canonical_defaults_and_modes_are_stable():
    source = (LAUNCH / 'observer.launch.py').read_text(encoding='utf-8')
    for value in (
        "default_value='full'", "default_value='standard'",
        "default_value='overview'", "default_value='false'",
    ):
        assert value in source
    assert "'rviz': 'rviz_observer.launch.py'" in source
    assert "'dashboard': 'dashboard_observer.launch.py'" in source
    assert "'full': 'full_observer.launch.py'" in source


def test_high_bandwidth_flags_are_forwarded_and_consumed():
    """Require independent image/cloud options to reach only RViz setup."""
    observer = (LAUNCH / 'observer.launch.py').read_text(encoding='utf-8')
    full = (LAUNCH / 'full_observer.launch.py').read_text(encoding='utf-8')
    rviz = (LAUNCH / 'rviz_observer.launch.py').read_text(encoding='utf-8')

    for argument in (
        'enable_camera_preview',
        'enable_pointclouds',
        'enable_raw_d435_pointcloud',
    ):
        for source in (observer, full, rviz):
            assert f"'{argument}', default_value='false'" in source

    for source in (observer, full, rviz):
        assert (
            "'d435_image_transport', default_value='compressed'"
            in source
        )

    for argument in (
        'enable_camera_preview',
        'enable_pointclouds',
        'enable_raw_d435_pointcloud',
        'd435_image_transport',
    ):
        assert f"'{argument}'," in observer
        assert f"'{argument}': LaunchConfiguration(" in full
        assert f"_value(context, '{argument}')" in rviz

    assert 'parse_launch_boolean' in rviz
    assert 'parse_d435_image_transport' in rviz
    assert 'or enable_raw_d435_pointcloud' in rviz
    assert "or d435_image_transport != 'compressed'" in rviz
    assert 'create_runtime_config' in rviz
    assert 'OnProcessExit' in rviz


def test_launches_contain_only_observer_and_rviz_processes():
    launch_files = (
        path
        for path in LAUNCH.glob('*')
        if path.is_file() and path.suffix != '.pyc'
    )
    source = '\n'.join(
        path.read_text(encoding='utf-8')
        for path in launch_files
    )
    for forbidden in (
        'nav2_bringup', 'slam_toolbox', 'savo_base', 'savo_control',
        'savo_supervisor', 'savo_mapping', 'savo_lidar', 'savo_realsense',
    ):
        assert forbidden not in source
    assert "package='rviz2'" in source
    assert "executable='observer_telemetry_node'" in source
    assert "executable='observer_dashboard_node'" in source
