# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_phase3_runtime_fixture_and_probe_are_present():
    fixture = (ROOT / 'test' / 'runtime' / 'phase2_fixture.py').read_text()
    probe = (ROOT / 'test' / 'runtime' / 'phase3_supervisor_probe.py').read_text()
    script = (ROOT / 'test' / 'runtime' /
              'run_phase3_edge_startup_test.sh').read_text()
    persistence_probe = (ROOT / 'test' / 'runtime' /
                         'phase3_persistence_probe.py').read_text()
    persistence_script = (ROOT / 'test' / 'runtime' /
                          'run_phase3_fault_persistence_test.sh').read_text()
    for topic in (
        '/savo_bridge/state',
        '/savo_bridge/readiness',
        '/savo_bridge/heartbeat',
        '/realsense/status',
        '/savo_speech/readiness',
        '/vo/health',
    ):
        assert topic in fixture
    assert 'COMMAND_CLEAR_FAULT_LATCH' in probe
    assert 'COMMAND_BEGIN_SHUTDOWN' in probe
    assert 'PHASE_3_EDGE_STARTUP_RUNTIME_COMPLETE' in probe
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in script
    assert 'ROS_LOCALHOST_ONLY' in script
    assert 'PHASE_3_FAULT_PERSISTENCE_RUNTIME_COMPLETE' in persistence_probe
    assert 'PHASE_3_FAULT_PERSISTENCE_RUNTIME_COMPLETE' in persistence_script
    assert 'system_state_path' in persistence_script
    assert (ROOT / 'test' / 'runtime' /
            'core_only_override.yaml').is_file()


def test_systemd_assets_are_installed_by_package():
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    unit = (ROOT / 'systemd' / 'savo-supervisor.service.in').read_text()
    installer = (ROOT / 'scripts' / 'install_core_runtime.sh').read_text()
    preflight = (ROOT / 'scripts' / 'real_robot_preflight.sh').read_text()
    validation = (ROOT / 'docs' / 'phase3_real_robot_validation.md').read_text()
    assert 'install(DIRECTORY systemd/' in cmake
    assert 'scripts/install_core_runtime.sh' in cmake
    assert 'scripts/real_robot_preflight.sh' in cmake
    assert 'install(DIRECTORY docs/' in cmake
    assert 'ProtectSystem=strict' in unit
    assert 'StateDirectory=robot_savo' in unit
    assert 'systemctl enable --now savo-supervisor.service' in installer
    assert 'PHASE_3_REAL_ROBOT_PREFLIGHT_COMPLETE' in preflight
    assert 'Phase 3 real-robot validation' in validation
