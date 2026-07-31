# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""LOC-3P-C semantic navigation runtime source contracts."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_semantic_navigation_runtime_is_approach_pose_only() -> None:
    """Semantic navigation forwards only the approved approach pose."""
    source = (ROOT / 'src/nodes/navigate_to_location_node.cpp').read_text()
    contract = (ROOT / 'src/core/location_navigation_contract.cpp').read_text()
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    fixture = (
        ROOT / 'test/fixtures/fake_location_nav2_server_node.cpp'
    ).read_text()
    smoke = (ROOT / 'scripts/run_location_integration_smoke').read_text()

    assert 'NavigateToLocation' in source
    assert '/savo_nav/locations/navigate' in source
    assert '/savo_locations/resolve' in source
    assert '/savo_supervisor/authorize_location_operation' in source
    assert 'OP_NAVIGATE_TO_LOCATION' in source
    assert 'OP_CONFIRM_LOCATION_ARRIVAL' in source
    assert 'navigation_goal.pose = location.approach_pose' in source
    assert 'authorization_recheck_period_s' in source
    assert 'async_cancel_goal(navigation_goal_handle)' in source
    assert 'ConfirmAprilTag::Goal::CONFIRM_ARRIVAL' in source

    assert 'tag_pose_map_must_never_be_navigation_target' in contract
    assert 'decision.target = record.approach_pose' in contract
    assert 'navigation_goal.pose = location.tag_pose_map' not in source
    assert 'navigate_to_location_node' in cmake
    assert 'fake_location_nav2_server_node' in cmake
    assert 'goal_publisher_->publish(goal_handle->get_goal()->pose)' in fixture
    assert 'ReviewLocationCandidate' in smoke
    assert '/savo_mapping/locations/review' in smoke
    assert 'location_review_gateway_node' in smoke
    assert '/savo_locations/candidates/approve' not in smoke
    assert 'LOC-3P-C2-C5 INTEGRATION SMOKE TEST: PASS' in smoke
    assert 'allocate_ros_domain_id' in smoke
    assert 'start_new_session=True' in smoke
    assert 'os.killpg' in smoke
    assert 'ROS_DOMAIN_ID = "98"' not in smoke
