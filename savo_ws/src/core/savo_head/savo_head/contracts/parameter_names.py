# -*- coding: utf-8 -*-

"""ROS parameter names for Robot Savo active head."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


HARDWARE_PROFILE: Final[str] = "hardware_profile"
BACKEND: Final[str] = "backend"
I2C_BUS: Final[str] = "i2c_bus"
PCA9685_ADDRESS: Final[str] = "pca9685_address"
PWM_FREQUENCY_HZ: Final[str] = "pwm_frequency_hz"

PAN_LOGICAL_CHANNEL: Final[str] = "pan_logical_channel"
TILT_LOGICAL_CHANNEL: Final[str] = "tilt_logical_channel"
PAN_PCA9685_CHANNEL: Final[str] = "pan_pca9685_channel"
TILT_PCA9685_CHANNEL: Final[str] = "tilt_pca9685_channel"
ALLOW_CHANNEL_OVERRIDE: Final[str] = "allow_channel_override"

SERVO_PULSE_MIN_US: Final[str] = "servo_pulse_min_us"
SERVO_PULSE_MAX_US: Final[str] = "servo_pulse_max_us"
SERVO_PERIOD_US: Final[str] = "servo_period_us"
PCA9685_TICKS_PER_CYCLE: Final[str] = "pca9685_ticks_per_cycle"
SERVO_ERROR_DEG: Final[str] = "servo_error_deg"

PAN_MIN_DEG: Final[str] = "pan_min_deg"
PAN_CENTER_DEG: Final[str] = "pan_center_deg"
PAN_MAX_DEG: Final[str] = "pan_max_deg"
TILT_MIN_DEG: Final[str] = "tilt_min_deg"
TILT_CENTER_DEG: Final[str] = "tilt_center_deg"
TILT_MAX_DEG: Final[str] = "tilt_max_deg"
MANUAL_STEP_DEG: Final[str] = "manual_step_deg"

CONTROL_HZ: Final[str] = "control_hz"
STATUS_HZ: Final[str] = "status_hz"
TF_HZ: Final[str] = "tf_hz"
WATCHDOG_TIMEOUT_S: Final[str] = "watchdog_timeout_s"

CENTER_ON_START: Final[str] = "center_on_start"
CENTER_ON_SHUTDOWN: Final[str] = "center_on_shutdown"
STOP_ON_WATCHDOG_TIMEOUT: Final[str] = "stop_on_watchdog_timeout"

DEFAULT_SCAN_PROFILE: Final[str] = "default_scan_profile"
SEMANTIC_SCAN_ENABLED: Final[str] = "semantic_scan_enabled"
SEMANTIC_SCAN_MODE: Final[str] = "semantic_scan_mode"

SEMANTIC_SCAN_PAN_MIN_DEG: Final[str] = "semantic_scan_pan_min_deg"
SEMANTIC_SCAN_PAN_CENTER_DEG: Final[str] = "semantic_scan_pan_center_deg"
SEMANTIC_SCAN_PAN_MAX_DEG: Final[str] = "semantic_scan_pan_max_deg"
SEMANTIC_SCAN_TILT_MIN_DEG: Final[str] = "semantic_scan_tilt_min_deg"
SEMANTIC_SCAN_TILT_MAX_DEG: Final[str] = "semantic_scan_tilt_max_deg"

SEMANTIC_SCAN_PAN_STEP_DEG: Final[str] = "semantic_scan_pan_step_deg"
SEMANTIC_SCAN_TILT_STEP_DEG: Final[str] = "semantic_scan_tilt_step_deg"
SEMANTIC_SCAN_STEP_DELAY_S: Final[str] = "semantic_scan_step_delay_s"

SEMANTIC_SCAN_START_PAN_DEG: Final[str] = "semantic_scan_start_pan_deg"
SEMANTIC_SCAN_START_TILT_DEG: Final[str] = "semantic_scan_start_tilt_deg"
SEMANTIC_SCAN_PAN_TARGETS_DEG: Final[str] = "semantic_scan_pan_targets_deg"
SEMANTIC_SCAN_TILT_SWEEP_PAN_TARGETS_DEG: Final[str] = (
    "semantic_scan_tilt_sweep_pan_targets_deg"
)

SEMANTIC_SCAN_HOLD_AT_PAN_TARGET_S: Final[str] = "semantic_scan_hold_at_pan_target_s"
SEMANTIC_SCAN_HOLD_AFTER_TILT_SWEEP_S: Final[str] = (
    "semantic_scan_hold_after_tilt_sweep_s"
)
SEMANTIC_SCAN_PAUSE_ON_MANUAL_COMMAND: Final[str] = (
    "semantic_scan_pause_on_manual_command"
)
SEMANTIC_SCAN_RESUME_AFTER_MANUAL_S: Final[str] = (
    "semantic_scan_resume_after_manual_s"
)
SEMANTIC_SCAN_CENTER_ON_STOP: Final[str] = "semantic_scan_center_on_stop"

PUBLISH_TF: Final[str] = "publish_tf"
PUBLISH_JOINT_STATE: Final[str] = "publish_joint_state"
TF_RATE_HZ: Final[str] = "tf_rate_hz"

BASE_FRAME: Final[str] = "base_frame"
PAN_FRAME: Final[str] = "pan_frame"
TILT_FRAME: Final[str] = "tilt_frame"
CAMERA_FRAME: Final[str] = "camera_frame"
CAMERA_OPTICAL_FRAME: Final[str] = "camera_optical_frame"

PAN_JOINT_NAME: Final[str] = "pan_joint_name"
TILT_JOINT_NAME: Final[str] = "tilt_joint_name"
PAN_AXIS: Final[str] = "pan_axis"
TILT_AXIS: Final[str] = "tilt_axis"
PAN_SIGN: Final[str] = "pan_sign"
TILT_SIGN: Final[str] = "tilt_sign"
PAN_ZERO_DEG: Final[str] = "pan_zero_deg"
TILT_ZERO_DEG: Final[str] = "tilt_zero_deg"

BASE_TO_PAN_XYZ_M: Final[str] = "base_to_pan_xyz_m"
BASE_TO_PAN_RPY_RAD: Final[str] = "base_to_pan_rpy_rad"
PAN_TO_TILT_XYZ_M: Final[str] = "pan_to_tilt_xyz_m"
PAN_TO_TILT_RPY_RAD: Final[str] = "pan_to_tilt_rpy_rad"
TILT_TO_CAMERA_XYZ_M: Final[str] = "tilt_to_camera_xyz_m"
TILT_TO_CAMERA_RPY_RAD: Final[str] = "tilt_to_camera_rpy_rad"
CAMERA_TO_OPTICAL_XYZ_M: Final[str] = "camera_to_optical_xyz_m"
CAMERA_TO_OPTICAL_RPY_RAD: Final[str] = "camera_to_optical_rpy_rad"

REQUIRE_VALID_PAN_TILT_STATE: Final[str] = "require_valid_pan_tilt_state"
STALE_STATE_TIMEOUT_S: Final[str] = "stale_state_timeout_s"

CAMERA_BACKEND: Final[str] = "camera_backend"
STREAM_ENABLED_ON_START: Final[str] = "stream_enabled_on_start"
REQUIRE_UDP_HOST: Final[str] = "require_udp_host"
UDP_HOST: Final[str] = "udp_host"
UDP_PORT: Final[str] = "udp_port"

WIDTH: Final[str] = "width"
HEIGHT: Final[str] = "height"
FPS: Final[str] = "fps"
FORMAT: Final[str] = "format"
BITRATE_KBPS: Final[str] = "bitrate_kbps"

GST_BINARY: Final[str] = "gst_binary"
GST_VERBOSE: Final[str] = "gst_verbose"
GST_SOURCE: Final[str] = "gst_source"
GST_CAPS_TEMPLATE: Final[str] = "gst_caps_template"
GST_USE_VIDEOCONVERT: Final[str] = "gst_use_videoconvert"
GST_ENCODER: Final[str] = "gst_encoder"
GST_ENCODER_TUNE: Final[str] = "gst_encoder_tune"
GST_ENCODER_SPEED_PRESET: Final[str] = "gst_encoder_speed_preset"
GST_ENCODER_KEY_INT_MAX: Final[str] = "gst_encoder_key_int_max"
GST_PAYLOADER: Final[str] = "gst_payloader"
GST_PAYLOADER_CONFIG_INTERVAL: Final[str] = "gst_payloader_config_interval"
GST_PAYLOADER_PT: Final[str] = "gst_payloader_pt"
GST_SINK: Final[str] = "gst_sink"
GST_SINK_SYNC: Final[str] = "gst_sink_sync"
GST_SINK_ASYNC: Final[str] = "gst_sink_async"
RECEIVER_CAPS: Final[str] = "receiver_caps"
RECEIVER_PIPELINE_HINT: Final[str] = "receiver_pipeline_hint"
PROCESS_START_TIMEOUT_S: Final[str] = "process_start_timeout_s"
PROCESS_STOP_TIMEOUT_S: Final[str] = "process_stop_timeout_s"
RESTART_ON_FAILURE: Final[str] = "restart_on_failure"

APRILTAG_ENABLED: Final[str] = "apriltag_enabled"
APRILTAG_FAMILY: Final[str] = "apriltag_family"
APRILTAG_DETECTIONS_TOPIC: Final[str] = "apriltag_detections_topic"
SEMANTIC_CONFIRMATIONS_TOPIC: Final[str] = "semantic_confirmations_topic"
ROBOT_BASE_FRAME: Final[str] = "robot_base_frame"
MAP_FRAME: Final[str] = "map_frame"
CONFIRMATION_SOURCE: Final[str] = "confirmation_source"
DEFAULT_CONFIRMATION_TYPE: Final[str] = "default_confirmation_type"
ALLOW_UNKNOWN_TAGS: Final[str] = "allow_unknown_tags"
REGISTERED_TAG_IDS_CSV: Final[str] = "registered_tag_ids_csv"
MIN_STABLE_FRAMES: Final[str] = "min_stable_frames"
MIN_DETECTION_CONFIDENCE: Final[str] = "min_detection_confidence"
MAX_DETECTION_DISTANCE_M: Final[str] = "max_detection_distance_m"
MAX_DETECTION_AGE_S: Final[str] = "max_detection_age_s"
REQUIRE_TF_AVAILABLE: Final[str] = "require_tf_available"
TF_TIMEOUT_S: Final[str] = "tf_timeout_s"
REQUIRE_ROBOT_STATIONARY: Final[str] = "require_robot_stationary"
MAX_ROBOT_LINEAR_SPEED_MPS: Final[str] = "max_robot_linear_speed_mps"
MAX_ROBOT_ANGULAR_SPEED_RADPS: Final[str] = "max_robot_angular_speed_radps"
REQUIRE_LOCALIZATION_OK: Final[str] = "require_localization_ok"
MAX_POSE_COVARIANCE_XY: Final[str] = "max_pose_covariance_xy"
MAX_YAW_COVARIANCE: Final[str] = "max_yaw_covariance"
REQUIRE_LIDAR_MAP_POSE: Final[str] = "require_lidar_map_pose"
REQUIRE_SEMANTIC_LABEL: Final[str] = "require_semantic_label"
SAVE_AS_SUMMON_POINT_BY_DEFAULT: Final[str] = "save_as_summon_point_by_default"
DUPLICATE_CONFIRM_DISTANCE_M: Final[str] = "duplicate_confirm_distance_m"
DUPLICATE_CONFIRM_YAW_RAD: Final[str] = "duplicate_confirm_yaw_rad"
COOLDOWN_AFTER_CONFIRMATION_S: Final[str] = "cooldown_after_confirmation_s"
TAG_PARAM_PREFIX: Final[str] = "tag_"

NAMESPACE_PARAM: Final[str] = "namespace"

PAN_TILT_CMD_TOPIC: Final[str] = "pan_tilt_cmd_topic"
PAN_TILT_STATE_TOPIC: Final[str] = "pan_tilt_state_topic"
SCAN_CMD_TOPIC: Final[str] = "scan_cmd_topic"
SCAN_STATE_TOPIC: Final[str] = "scan_state_topic"
STATUS_TOPIC_PARAM: Final[str] = "status_topic"
DASHBOARD_TEXT_TOPIC_PARAM: Final[str] = "dashboard_text_topic"
CAMERA_STREAM_CMD_TOPIC: Final[str] = "camera_stream_cmd_topic"
CAMERA_STREAM_STATE_TOPIC: Final[str] = "camera_stream_state_topic"
CAMERA_STATUS_TOPIC: Final[str] = "camera_status_topic"
IMAGE_RAW_TOPIC: Final[str] = "image_raw_topic"
CAMERA_INFO_TOPIC: Final[str] = "camera_info_topic"
EMERGENCY_CENTER_TOPIC: Final[str] = "emergency_center_topic"

HEALTH_CHECK_SERVICE: Final[str] = "health_check_service"
CENTER_SERVICE: Final[str] = "center_service"
SET_SCAN_MODE_SERVICE: Final[str] = "set_scan_mode_service"

PAN_TILT_CMD_TYPE: Final[str] = "pan_tilt_cmd_type"
PAN_TILT_STATE_TYPE: Final[str] = "pan_tilt_state_type"
SCAN_CMD_TYPE: Final[str] = "scan_cmd_type"
SCAN_STATE_TYPE: Final[str] = "scan_state_type"
STATUS_TYPE: Final[str] = "status_type"
DASHBOARD_TEXT_TYPE: Final[str] = "dashboard_text_type"

COMMAND_QOS_DEPTH: Final[str] = "command_qos_depth"
STATE_QOS_DEPTH: Final[str] = "state_qos_depth"
SENSOR_QOS_DEPTH: Final[str] = "sensor_qos_depth"
STATUS_QOS_DEPTH: Final[str] = "status_qos_depth"

COMMAND_RELIABILITY: Final[str] = "command_reliability"
STATE_RELIABILITY: Final[str] = "state_reliability"
SENSOR_RELIABILITY: Final[str] = "sensor_reliability"
STATUS_RELIABILITY: Final[str] = "status_reliability"

TAG_PARAM_PREFIX: Final[str] = "tag_param_prefix"

DIAGNOSTICS_ENABLED: Final[str] = "diagnostics_enabled"
DIAGNOSTICS_TOPIC: Final[str] = "diagnostics_topic"
STATUS_TOPIC: Final[str] = "status_topic"
DASHBOARD_TEXT_TOPIC: Final[str] = "dashboard_text_topic"
HARDWARE_STATUS_NAME: Final[str] = "hardware_status_name"
SCAN_STATUS_NAME: Final[str] = "scan_status_name"
TF_STATUS_NAME: Final[str] = "tf_status_name"
CAMERA_STATUS_NAME: Final[str] = "camera_status_name"
APRILTAG_STATUS_NAME: Final[str] = "apriltag_status_name"
STATUS_PUBLISH_HZ: Final[str] = "status_publish_hz"
DASHBOARD_PUBLISH_HZ: Final[str] = "dashboard_publish_hz"
COMMAND_STALE_TIMEOUT_S: Final[str] = "command_stale_timeout_s"
STATE_STALE_TIMEOUT_S: Final[str] = "state_stale_timeout_s"
TF_STALE_TIMEOUT_S: Final[str] = "tf_stale_timeout_s"
CAMERA_STREAM_STALE_TIMEOUT_S: Final[str] = "camera_stream_stale_timeout_s"
APRILTAG_DETECTION_STALE_TIMEOUT_S: Final[str] = "apriltag_detection_stale_timeout_s"
WARN_SERVO_ERROR_DEG: Final[str] = "warn_servo_error_deg"
ERROR_SERVO_ERROR_DEG: Final[str] = "error_servo_error_deg"
WARN_I2C_FAILURE_COUNT: Final[str] = "warn_i2c_failure_count"
ERROR_I2C_FAILURE_COUNT: Final[str] = "error_i2c_failure_count"
WARN_CAMERA_RESTART_COUNT: Final[str] = "warn_camera_restart_count"
ERROR_CAMERA_RESTART_COUNT: Final[str] = "error_camera_restart_count"
WARN_TF_DROP_COUNT: Final[str] = "warn_tf_drop_count"
ERROR_TF_DROP_COUNT: Final[str] = "error_tf_drop_count"
REQUIRE_HARDWARE_FOR_OK: Final[str] = "require_hardware_for_ok"
REQUIRE_TF_FOR_OK: Final[str] = "require_tf_for_ok"
REQUIRE_CAMERA_FOR_OK: Final[str] = "require_camera_for_ok"
REQUIRE_APRILTAG_FOR_OK: Final[str] = "require_apriltag_for_ok"
DRYRUN_IS_WARNING: Final[str] = "dryrun_is_warning"
PUBLISH_OK_WHEN_IDLE: Final[str] = "publish_ok_when_idle"


@dataclass(frozen=True)
class HeadParameterNames:
    interfaces: Tuple[str, ...] = (
        NAMESPACE_PARAM,
        PAN_TILT_CMD_TOPIC,
        PAN_TILT_STATE_TOPIC,
        SCAN_CMD_TOPIC,
        SCAN_STATE_TOPIC,
        STATUS_TOPIC_PARAM,
        DASHBOARD_TEXT_TOPIC_PARAM,
        CAMERA_STREAM_CMD_TOPIC,
        CAMERA_STREAM_STATE_TOPIC,
        CAMERA_STATUS_TOPIC,
        IMAGE_RAW_TOPIC,
        CAMERA_INFO_TOPIC,
        EMERGENCY_CENTER_TOPIC,
        HEALTH_CHECK_SERVICE,
        CENTER_SERVICE,
        SET_SCAN_MODE_SERVICE,
        PAN_TILT_CMD_TYPE,
        PAN_TILT_STATE_TYPE,
        SCAN_CMD_TYPE,
        SCAN_STATE_TYPE,
        STATUS_TYPE,
        DASHBOARD_TEXT_TYPE,
        COMMAND_QOS_DEPTH,
        STATE_QOS_DEPTH,
        SENSOR_QOS_DEPTH,
        STATUS_QOS_DEPTH,
        COMMAND_RELIABILITY,
        STATE_RELIABILITY,
        SENSOR_RELIABILITY,
        STATUS_RELIABILITY,
    )

    hardware: Tuple[str, ...] = (
        HARDWARE_PROFILE,
        BACKEND,
        I2C_BUS,
        PCA9685_ADDRESS,
        PWM_FREQUENCY_HZ,
        PAN_LOGICAL_CHANNEL,
        TILT_LOGICAL_CHANNEL,
        PAN_PCA9685_CHANNEL,
        TILT_PCA9685_CHANNEL,
        ALLOW_CHANNEL_OVERRIDE,
        SERVO_PULSE_MIN_US,
        SERVO_PULSE_MAX_US,
        SERVO_PERIOD_US,
        PCA9685_TICKS_PER_CYCLE,
        SERVO_ERROR_DEG,
        PAN_MIN_DEG,
        PAN_CENTER_DEG,
        PAN_MAX_DEG,
        TILT_MIN_DEG,
        TILT_CENTER_DEG,
        TILT_MAX_DEG,
        MANUAL_STEP_DEG,
        CONTROL_HZ,
        STATUS_HZ,
        TF_HZ,
        WATCHDOG_TIMEOUT_S,
        CENTER_ON_START,
        CENTER_ON_SHUTDOWN,
        STOP_ON_WATCHDOG_TIMEOUT,
    )

    scan: Tuple[str, ...] = (
        DEFAULT_SCAN_PROFILE,
        SEMANTIC_SCAN_ENABLED,
        SEMANTIC_SCAN_MODE,
        SEMANTIC_SCAN_PAN_MIN_DEG,
        SEMANTIC_SCAN_PAN_CENTER_DEG,
        SEMANTIC_SCAN_PAN_MAX_DEG,
        SEMANTIC_SCAN_TILT_MIN_DEG,
        SEMANTIC_SCAN_TILT_MAX_DEG,
        SEMANTIC_SCAN_PAN_STEP_DEG,
        SEMANTIC_SCAN_TILT_STEP_DEG,
        SEMANTIC_SCAN_STEP_DELAY_S,
        SEMANTIC_SCAN_START_PAN_DEG,
        SEMANTIC_SCAN_START_TILT_DEG,
        SEMANTIC_SCAN_PAN_TARGETS_DEG,
        SEMANTIC_SCAN_TILT_SWEEP_PAN_TARGETS_DEG,
        SEMANTIC_SCAN_HOLD_AT_PAN_TARGET_S,
        SEMANTIC_SCAN_HOLD_AFTER_TILT_SWEEP_S,
        SEMANTIC_SCAN_PAUSE_ON_MANUAL_COMMAND,
        SEMANTIC_SCAN_RESUME_AFTER_MANUAL_S,
        SEMANTIC_SCAN_CENTER_ON_STOP,
    )

    frames: Tuple[str, ...] = (
        PUBLISH_TF,
        PUBLISH_JOINT_STATE,
        TF_RATE_HZ,
        BASE_FRAME,
        PAN_FRAME,
        TILT_FRAME,
        CAMERA_FRAME,
        CAMERA_OPTICAL_FRAME,
        PAN_JOINT_NAME,
        TILT_JOINT_NAME,
        PAN_AXIS,
        TILT_AXIS,
        PAN_SIGN,
        TILT_SIGN,
        PAN_ZERO_DEG,
        TILT_ZERO_DEG,
        BASE_TO_PAN_XYZ_M,
        BASE_TO_PAN_RPY_RAD,
        PAN_TO_TILT_XYZ_M,
        PAN_TO_TILT_RPY_RAD,
        TILT_TO_CAMERA_XYZ_M,
        TILT_TO_CAMERA_RPY_RAD,
        CAMERA_TO_OPTICAL_XYZ_M,
        CAMERA_TO_OPTICAL_RPY_RAD,
        REQUIRE_VALID_PAN_TILT_STATE,
        STALE_STATE_TIMEOUT_S,
    )

    camera: Tuple[str, ...] = (
        CAMERA_BACKEND,
        STREAM_ENABLED_ON_START,
        REQUIRE_UDP_HOST,
        UDP_HOST,
        UDP_PORT,
        WIDTH,
        HEIGHT,
        FPS,
        FORMAT,
        BITRATE_KBPS,
        GST_BINARY,
        GST_VERBOSE,
        GST_SOURCE,
        GST_CAPS_TEMPLATE,
        GST_USE_VIDEOCONVERT,
        GST_ENCODER,
        GST_ENCODER_TUNE,
        GST_ENCODER_SPEED_PRESET,
        GST_ENCODER_KEY_INT_MAX,
        GST_PAYLOADER,
        GST_PAYLOADER_CONFIG_INTERVAL,
        GST_PAYLOADER_PT,
        GST_SINK,
        GST_SINK_SYNC,
        GST_SINK_ASYNC,
        RECEIVER_CAPS,
        RECEIVER_PIPELINE_HINT,
        PROCESS_START_TIMEOUT_S,
        PROCESS_STOP_TIMEOUT_S,
        RESTART_ON_FAILURE,
    )

    apriltag: Tuple[str, ...] = (
        APRILTAG_ENABLED,
        APRILTAG_FAMILY,
        APRILTAG_DETECTIONS_TOPIC,
        SEMANTIC_CONFIRMATIONS_TOPIC,
        CAMERA_OPTICAL_FRAME,
        ROBOT_BASE_FRAME,
        MAP_FRAME,
        CONFIRMATION_SOURCE,
        DEFAULT_CONFIRMATION_TYPE,
        ALLOW_UNKNOWN_TAGS,
        REGISTERED_TAG_IDS_CSV,
        MIN_STABLE_FRAMES,
        MIN_DETECTION_CONFIDENCE,
        MAX_DETECTION_DISTANCE_M,
        MAX_DETECTION_AGE_S,
        REQUIRE_TF_AVAILABLE,
        TF_TIMEOUT_S,
        REQUIRE_ROBOT_STATIONARY,
        MAX_ROBOT_LINEAR_SPEED_MPS,
        MAX_ROBOT_ANGULAR_SPEED_RADPS,
        REQUIRE_LOCALIZATION_OK,
        MAX_POSE_COVARIANCE_XY,
        MAX_YAW_COVARIANCE,
        REQUIRE_LIDAR_MAP_POSE,
        REQUIRE_SEMANTIC_LABEL,
        SAVE_AS_SUMMON_POINT_BY_DEFAULT,
        DUPLICATE_CONFIRM_DISTANCE_M,
        DUPLICATE_CONFIRM_YAW_RAD,
        COOLDOWN_AFTER_CONFIRMATION_S,
        TAG_PARAM_PREFIX,
    )

    diagnostics: Tuple[str, ...] = (
        DIAGNOSTICS_ENABLED,
        DIAGNOSTICS_TOPIC,
        STATUS_TOPIC,
        DASHBOARD_TEXT_TOPIC,
        HARDWARE_STATUS_NAME,
        SCAN_STATUS_NAME,
        TF_STATUS_NAME,
        CAMERA_STATUS_NAME,
        APRILTAG_STATUS_NAME,
        STATUS_PUBLISH_HZ,
        DASHBOARD_PUBLISH_HZ,
        COMMAND_STALE_TIMEOUT_S,
        STATE_STALE_TIMEOUT_S,
        TF_STALE_TIMEOUT_S,
        CAMERA_STREAM_STALE_TIMEOUT_S,
        APRILTAG_DETECTION_STALE_TIMEOUT_S,
        WARN_SERVO_ERROR_DEG,
        ERROR_SERVO_ERROR_DEG,
        WARN_I2C_FAILURE_COUNT,
        ERROR_I2C_FAILURE_COUNT,
        WARN_CAMERA_RESTART_COUNT,
        ERROR_CAMERA_RESTART_COUNT,
        WARN_TF_DROP_COUNT,
        ERROR_TF_DROP_COUNT,
        REQUIRE_HARDWARE_FOR_OK,
        REQUIRE_TF_FOR_OK,
        REQUIRE_CAMERA_FOR_OK,
        REQUIRE_APRILTAG_FOR_OK,
        DRYRUN_IS_WARNING,
        PUBLISH_OK_WHEN_IDLE,
    )

    def grouped(self) -> Tuple[Tuple[str, ...], ...]:
        return (
            self.interfaces,
            self.hardware,
            self.scan,
            self.frames,
            self.camera,
            self.apriltag,
            self.diagnostics,
        )

    def all_raw(self) -> Tuple[str, ...]:
        out: Tuple[str, ...] = ()
        for group in self.grouped():
            out += group
        return out

    def all(self) -> Tuple[str, ...]:
        seen = set()
        out = []
        for name in self.all_raw():
            if name not in seen:
                seen.add(name)
                out.append(name)
        return tuple(out)


PARAMETERS: Final[HeadParameterNames] = HeadParameterNames()


def get_parameter_names() -> HeadParameterNames:
    return PARAMETERS


def is_known_parameter(name: str) -> bool:
    return str(name) in PARAMETERS.all()


__all__ = [
    name
    for name in tuple(globals())
    if name.isupper()
] + [
    "HeadParameterNames",
    "PARAMETERS",
    "get_parameter_names",
    "is_known_parameter",
]
