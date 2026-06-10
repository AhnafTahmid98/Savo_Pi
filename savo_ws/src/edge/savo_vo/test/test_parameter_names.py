"""Tests for savo_vo parameter name contracts."""

from savo_vo.contracts.parameter_names import (
    BASE_FRAME_PARAM,
    CAMERA_FRAME_PARAM,
    COLOR_CAMERA_INFO_TOPIC_PARAM,
    COLOR_IMAGE_TOPIC_PARAM,
    DEPTH_CAMERA_INFO_TOPIC_PARAM,
    DEPTH_IMAGE_TOPIC_PARAM,
    DIAGNOSTICS_RATE_HZ_PARAM,
    GOOD_FEATURES_TARGET_PARAM,
    HEALTH_TOPIC_PARAM,
    MAX_DEPTH_DELAY_S_PARAM,
    MAX_FEATURES_PARAM,
    MAX_IMAGE_DELAY_S_PARAM,
    MAX_ROTATION_JUMP_RAD_PARAM,
    MAX_TRANSLATION_JUMP_M_PARAM,
    MIN_FEATURES_PARAM,
    MIN_TRACKING_QUALITY_PARAM,
    ODOM_FRAME_PARAM,
    ODOM_RAW_TOPIC_PARAM,
    ODOM_TOPIC_PARAM,
    PUBLISH_DIAGNOSTICS_PARAM,
    PUBLISH_RATE_HZ_PARAM,
    PUBLISH_TF_PARAM,
    STALE_TIMEOUT_S_PARAM,
    STATUS_TOPIC_PARAM,
    TRACKING_QUALITY_TOPIC_PARAM,
)


def test_topic_parameter_names_match_expected_defaults() -> None:
    assert COLOR_IMAGE_TOPIC_PARAM == "color_image_topic"
    assert COLOR_CAMERA_INFO_TOPIC_PARAM == "color_camera_info_topic"
    assert DEPTH_IMAGE_TOPIC_PARAM == "depth_image_topic"
    assert DEPTH_CAMERA_INFO_TOPIC_PARAM == "depth_camera_info_topic"

    assert ODOM_TOPIC_PARAM == "odom_topic"
    assert ODOM_RAW_TOPIC_PARAM == "odom_raw_topic"
    assert STATUS_TOPIC_PARAM == "status_topic"
    assert HEALTH_TOPIC_PARAM == "health_topic"
    assert TRACKING_QUALITY_TOPIC_PARAM == "tracking_quality_topic"


def test_frame_parameter_names_match_expected_defaults() -> None:
    assert ODOM_FRAME_PARAM == "odom_frame"
    assert BASE_FRAME_PARAM == "base_frame"
    assert CAMERA_FRAME_PARAM == "camera_frame"


def test_rate_and_timeout_parameter_names_match_expected_defaults() -> None:
    assert PUBLISH_RATE_HZ_PARAM == "publish_rate_hz"
    assert DIAGNOSTICS_RATE_HZ_PARAM == "diagnostics_rate_hz"

    assert MAX_IMAGE_DELAY_S_PARAM == "max_image_delay_s"
    assert MAX_DEPTH_DELAY_S_PARAM == "max_depth_delay_s"
    assert STALE_TIMEOUT_S_PARAM == "stale_timeout_s"


def test_tracking_parameter_names_match_expected_defaults() -> None:
    assert MIN_FEATURES_PARAM == "min_features"
    assert GOOD_FEATURES_TARGET_PARAM == "good_features_target"
    assert MAX_FEATURES_PARAM == "max_features"

    assert MIN_TRACKING_QUALITY_PARAM == "min_tracking_quality"
    assert MAX_TRANSLATION_JUMP_M_PARAM == "max_translation_jump_m"
    assert MAX_ROTATION_JUMP_RAD_PARAM == "max_rotation_jump_rad"


def test_boolean_parameter_names_match_expected_defaults() -> None:
    assert PUBLISH_TF_PARAM == "publish_tf"
    assert PUBLISH_DIAGNOSTICS_PARAM == "publish_diagnostics"


def test_parameter_names_are_valid_ros_parameter_keys() -> None:
    parameter_names = [
        COLOR_IMAGE_TOPIC_PARAM,
        COLOR_CAMERA_INFO_TOPIC_PARAM,
        DEPTH_IMAGE_TOPIC_PARAM,
        DEPTH_CAMERA_INFO_TOPIC_PARAM,
        ODOM_TOPIC_PARAM,
        ODOM_RAW_TOPIC_PARAM,
        STATUS_TOPIC_PARAM,
        HEALTH_TOPIC_PARAM,
        TRACKING_QUALITY_TOPIC_PARAM,
        ODOM_FRAME_PARAM,
        BASE_FRAME_PARAM,
        CAMERA_FRAME_PARAM,
        PUBLISH_RATE_HZ_PARAM,
        DIAGNOSTICS_RATE_HZ_PARAM,
        MAX_IMAGE_DELAY_S_PARAM,
        MAX_DEPTH_DELAY_S_PARAM,
        STALE_TIMEOUT_S_PARAM,
        MIN_FEATURES_PARAM,
        GOOD_FEATURES_TARGET_PARAM,
        MAX_FEATURES_PARAM,
        MIN_TRACKING_QUALITY_PARAM,
        MAX_TRANSLATION_JUMP_M_PARAM,
        MAX_ROTATION_JUMP_RAD_PARAM,
        PUBLISH_TF_PARAM,
        PUBLISH_DIAGNOSTICS_PARAM,
    ]

    for name in parameter_names:
        assert name
        assert name.strip() == name
        assert " " not in name
        assert name.lower() == name