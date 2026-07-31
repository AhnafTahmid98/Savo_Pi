# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PACKAGE_ROOT / 'scripts' / 'run_mapping_nav_preflight'
CMAKE = PACKAGE_ROOT / 'CMakeLists.txt'


def test_preflight_is_installed_and_executable() -> None:
    assert SCRIPT.is_file()
    assert SCRIPT.stat().st_mode & 0o111
    assert 'scripts/run_mapping_nav_preflight' in CMAKE.read_text()


def test_preflight_is_observation_only() -> None:
    text = SCRIPT.read_text()
    forbidden = (
        'ros2 topic pub',
        'ros2 action send_goal',
        'ros2 lifecycle set',
        'ros2 service call',
    )
    for token in forbidden:
        assert token not in text


def test_preflight_checks_live_and_saved_map_ownership() -> None:
    text = SCRIPT.read_text()
    assert '--mode live|saved' in text
    assert '/amcl' in text
    assert '/map_server' in text
    assert 'AMCL is running during live mapping' in text
    assert 'map_server is running during live mapping' in text


def test_preflight_checks_navigation_dependencies() -> None:
    text = SCRIPT.read_text()
    required = (
        '/tf',
        '/tf_static',
        '/map',
        '/scan',
        '/odometry/filtered',
        '/safety/stop',
        '/safety/slowdown_factor',
        '/savo_control/mode_state',
        '/global_costmap/costmap',
        '/local_costmap/costmap',
        '/savo_nav/readiness',
        '/savo_nav/control_recovery/allowed',
        '/savo_nav/goal_admission/state',
        '/navigate_to_pose',
        '/savo_nav/navigation/navigate_to_pose',
        '/savo_nav/exploration/navigate_to_pose',
        '/savo_nav/coverage/execute_path',
    )
    for token in required:
        assert token in text


def test_preflight_runtime_dependencies_are_declared() -> None:
    package_xml = (PACKAGE_ROOT / 'package.xml').read_text()
    for dependency in ('ros2action', 'ros2cli', 'ros2node', 'ros2topic'):
        assert f'<exec_depend>{dependency}</exec_depend>' in package_xml
