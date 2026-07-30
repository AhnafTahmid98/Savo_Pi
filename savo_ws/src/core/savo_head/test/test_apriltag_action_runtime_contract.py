# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: Apache-2.0

"""LOC-3P-C typed AprilTag runtime source contracts."""

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
