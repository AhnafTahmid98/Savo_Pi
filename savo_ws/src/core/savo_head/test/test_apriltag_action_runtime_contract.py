# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: Apache-2.0

"""LOC-3P-C typed AprilTag runtime source contracts."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_typed_observation_bridge_and_action_server_are_wired() -> None:
    """Bridge typed observations and expose a stable confirmation action."""
    legacy = (ROOT / "src/nodes/apriltag_confirm_node.cpp").read_text()
    action = (ROOT / "src/nodes/apriltag_confirmation_action_node.cpp").read_text()
    cmake = (ROOT / "CMakeLists.txt").read_text()

    assert "typed_observations_topic" in legacy
    assert "savo_msgs::msg::AprilTagObservation" in legacy
    assert "publish_typed_observation(observation)" in legacy
    assert "/savo_head/apriltag/observations" in legacy

    assert "rclcpp_action::create_server<ConfirmAprilTag>" in action
    assert "RESULT_WRONG_TAG" in action
    assert "RESULT_UNSTABLE" in action
    assert "RESULT_CANCELED" in action
    assert "minimum_observations" in action
    assert "maximum_observation_age_s" in action
    assert "tf_buffer_.transform" in action
    assert "require_stationary_signal" in action
    assert "require_localization_signal" in action

    assert "apriltag_confirmation_action_node" in cmake
    assert "tf2_geometry_msgs" in cmake


def test_identity_only_arrival_does_not_consume_fake_spatial_evidence() -> None:
    """Pose-less arrival uses real identity observations and no fake map pose."""
    action = (ROOT / "src/nodes/apriltag_confirmation_action_node.cpp").read_text()

    assert "std::vector<Observation> accepted;" in action
    assert "std::vector<SpatialSample> spatial_samples;" in action
    assert "result->final_observation = accepted.back();" in action
    assert "apriltag_identity_confirmation_succeeded" in action
    assert re.search(
        r"apriltag_identity_confirmation_succeeded.*?accepted,.*?rejected,"
        r".*?std::nullopt,.*?false",
        action,
        re.DOTALL,
    )

    # The common freshness/quality filter must not reject pose-less identity.
    common_filter = action.split("const rclcpp::Time observation_stamp", 1)[1]
    common_filter = common_filter.split("if (spatial_evidence_required)", 1)[0]
    assert "pose_valid" not in common_filter


def test_spatial_paths_still_require_pose_tf_and_stability() -> None:
    """Map-pose arrival and registration retain their spatial safeguards."""
    action = (ROOT / "src/nodes/apriltag_confirmation_action_node.cpp").read_text()

    spatial_block = action.split("if (spatial_evidence_required)", 1)[1]
    spatial_block = spatial_block.split("accepted.push_back(observation)", 1)[0]
    for required in (
        "if (!observation.pose_valid)",
        "map_pose(observation, true)",
        "if (!transformed.has_value())",
        "spatial_samples.push_back",
    ):
        assert required in spatial_block

    assert "summarize(\n            spatial_samples" in action
    assert "only_unexpected_apriltag_observed" in action
