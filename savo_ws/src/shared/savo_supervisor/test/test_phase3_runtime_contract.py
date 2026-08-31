# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_phase3_core_sources_are_built():
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    for source in (
        'src/edge_payload_parser.cpp',
        'src/edge_supervision.cpp',
        'src/system_authority.cpp',
        'src/system_state_store.cpp',
    ):
        assert source in cmake


def test_supervisor_exposes_system_authority_and_edge_state():
    node = (ROOT / 'src' / 'supervisor_node.cpp').read_text()
    for token in (
        'ManageSystemState',
        'create_edge_subscriptions',
        'edge_capabilities',
        'system_authority',
        'remote_commands_ready',
        'controlled_shutdown_requested',
        'operation_authorization_revoked',
        'system_actor_not_authorized',
    ):
        assert token in node


def test_remote_missions_fail_closed_when_bridge_path_is_lost():
    authority = (ROOT / 'src' / 'mission_authority.cpp').read_text()
    assert 'request.remote_origin' in authority
    assert 'remote_command_path_not_ready' in authority
    assert 'system_not_armed' in authority
    assert 'system_fault_latched' in authority
    assert 'system_shutting_down' in authority


def test_invalid_phase3_configuration_fails_closed():
    node = (ROOT / 'src' / 'supervisor_node.cpp').read_text()
    assert 'required edge component cannot be disabled' in node
    assert 'duplicate supervisor output topic' in node
    assert 'invalid edge supervision configuration' in node
    assert 'persistent_state_invalid:' in node


def test_production_config_distinguishes_required_and_optional_edge_capabilities():
    config = (ROOT / 'config' / 'supervisor.yaml').read_text()
    assert 'edge.bridge.required_for_startup: false' in config
    assert 'edge.realsense.required_for_startup: false' in config
    assert 'edge.speech.required_for_startup: false' in config
    assert 'edge.vo.required_for_startup: false' in config
    assert 'edge.ui.required_for_startup: false' in config
    assert 'system_authority.auto_arm: false' in config


def test_periodic_realsense_and_vo_health_use_production_volatile_qos():
    node = (ROOT / 'src' / 'supervisor_node.cpp').read_text()
    fixture = (ROOT / 'test' / 'runtime' / 'phase2_fixture.py').read_text()
    realsense = (
        ROOT.parents[1] / 'edge' / 'savo_realsense' / 'src' /
        'camera_health_main.cpp'
    ).read_text()
    vo = (
        ROOT.parents[1] / 'edge' / 'savo_vo' / 'src' / 'vo_health_node.cpp'
    ).read_text()

    expected_qos = 'rclcpp::QoS(10).reliable().durability_volatile()'
    assert node.count(expected_qos) >= 2
    assert 'realsense_status_topic_, ' + expected_qos in node
    assert 'vo_health_topic_, ' + expected_qos in node
    realsense_qos = (
        'auto reliable_qos = '
        'rclcpp::QoS(rclcpp::KeepLast(10)).reliable()'
    )
    assert realsense_qos in realsense
    assert 'qos.durability_volatile()' in vo
    assert 'return "stale: visual odometry timeout";' in vo
    assert "String, '/realsense/status', status_qos" in fixture
    assert "String, '/vo/health', status_qos" in fixture
