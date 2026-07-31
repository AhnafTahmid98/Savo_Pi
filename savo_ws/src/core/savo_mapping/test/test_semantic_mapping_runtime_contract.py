# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Semantic mapping runtime ownership and deployment contracts."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_semantic_bridge_is_read_only_and_typed_at_registry_boundary() -> None:
    """The bridge emits prompts/events but owns no persistence or motion."""
    source = (
        ROOT / 'src/nodes/semantic_landmark_bridge_node.cpp'
    ).read_text()

    required = (
        'SemanticLandmarkBridgeNode',
        'topics::LOCATION_EVENTS',
        'savo_msgs::msg::LocationEvent',
        'topics::HEAD_SEMANTIC_CONFIRMATIONS',
        'topics::SEMANTIC_EVENTS',
        'topics::SEMANTIC_STATUS',
        'topics::SEMANTIC_HEARTBEAT',
        'apriltag_candidate_observed',
        'operator_action_required',
        'evidence_is_hint_only',
        'persistence_authority',
        'savo_locations',
        'registration_action',
        '/savo_mapping/locations/register',
        'duplicate_head_confirmation_suppressed',
    )
    for token in required:
        assert token in source

    forbidden = (
        'sqlite3_open',
        'sqlite3_exec',
        'create_client<',
        'create_service<',
        '/cmd_vel',
        '/navigate_to_pose',
        'async_send_goal',
    )
    for token in forbidden:
        assert token not in source


def test_semantic_launch_starts_mapping_registration_and_event_bridge() -> None:
    """The semantic entrypoint starts mapping and both mapping-owned nodes."""
    launch = (ROOT / 'launch/semantic_mapping.launch.xml').read_text()
    config = (ROOT / 'config/semantic_landmarks.yaml').read_text()
    manual_launch = (ROOT / 'launch/manual_mapping.launch.xml').read_text()

    assert 'manual_mapping.launch.xml' in launch
    assert 'semantic_mapping_enabled' in launch
    assert 'value="true"' in launch
    assert 'mapped_location_registration_node' in launch
    assert 'semantic_landmark_bridge_node' in launch
    assert 'semantic_landmarks.yaml' in launch

    assert 'optional_features.semantic_mapping_enabled' in manual_launch
    assert 'head_confirmation_action: /savo_head/apriltag/confirm' in config
    assert 'registration_service: /savo_locations/candidates/register' in config
    assert 'location_events_topic: /savo_locations/events' in config
    assert 'semantic_events_topic: /savo_mapping/semantic_events' in config
    assert 'require_active_mapping_session: true' in config


def test_semantic_assets_are_installed_and_supervisor_accepts_feature() -> None:
    """The CMake configuration installs and tests the implementation."""
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    runtime = (ROOT / 'src/core/supervisor_runtime.cpp').read_text()

    assert 'semantic_landmark_bridge_node' in cmake
    assert 'src/nodes/semantic_landmark_bridge_node.cpp' in cmake
    assert 'config/semantic_landmarks.yaml' in cmake
    assert 'launch/semantic_mapping.launch.xml' in cmake
    assert 'test_semantic_mapping_runtime_contract' in cmake

    assert 'semantic_mapping_not_implemented' not in runtime
    assert 'Semantic mapping is a read-only integration' in runtime
