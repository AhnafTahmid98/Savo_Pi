# -*- coding: utf-8 -*-

"""Package-wide constants. No ROS imports — safe to import anywhere."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Mapping, Tuple


# =============================================================================
# Package / Identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_head"
ROBOT_NAME: Final[str] = "Robot Savo"

NODE_NAME_HEAD_CONTROLLER: Final[str] = "head_controller_node"
NODE_NAME_HEAD_SCAN: Final[str] = "head_scan_node"
NODE_NAME_HEAD_TF: Final[str] = "head_tf_node"
NODE_NAME_HEAD_STATUS: Final[str] = "head_status_node"
NODE_NAME_APRILTAG_CONFIRM: Final[str] = "apriltag_confirm_node"


# =============================================================================
# Topic Names
# =============================================================================
TOPIC_PAN_TILT_CMD: Final[str] = "/savo_head/pan_tilt_cmd"
TOPIC_PAN_TILT_STATE: Final[str] = "/savo_head/pan_tilt_state"

TOPIC_SCAN_CMD: Final[str] = "/savo_head/scan_cmd"
TOPIC_SCAN_STATE: Final[str] = "/savo_head/scan_state"

TOPIC_STATUS: Final[str] = "/savo_head/status"
TOPIC_DASHBOARD_TEXT: Final[str] = "/savo_head/dashboard_text"

TOPIC_IMAGE_RAW: Final[str] = "/savo_head/camera/image_raw"
TOPIC_CAMERA_INFO: Final[str] = "/savo_head/camera/camera_info"
TOPIC_CAMERA_STATUS: Final[str] = "/savo_head/camera/status"

TOPIC_APRILTAG_DETECTIONS: Final[str] = "/savo_head/apriltag_detections"
TOPIC_SEMANTIC_CONFIRMATIONS: Final[str] = "/savo_head/semantic_confirmations"


# =============================================================================
# TF Frames
# =============================================================================
FRAME_BASE: Final[str] = "base_link"
FRAME_PANTILT_PAN: Final[str] = "pantilt_pan_link"
FRAME_PANTILT_TILT: Final[str] = "pantilt_tilt_link"
FRAME_PI_CAMERA: Final[str] = "pi_camera_link"
FRAME_PI_CAMERA_OPTICAL: Final[str] = "pi_camera_optical_frame"


# =============================================================================
# Hardware Backend / Board Defaults
# =============================================================================
HEAD_BACKEND_DEFAULT: Final[str] = "pca9685"  # pca9685 | dryrun
HEAD_HARDWARE_PROFILE_DEFAULT: Final[str] = "robot_savo_core_v1"

I2C_BUS_DEFAULT: Final[int] = 1
PCA9685_ADDR_DEFAULT: Final[int] = 0x40
PCA9685_PWM_FREQ_HZ_DEFAULT: Final[float] = 50.0

PCA9685_PULSE_MIN_US: Final[float] = 500.0
PCA9685_PULSE_MAX_US: Final[float] = 2500.0
PCA9685_SERVO_PERIOD_US: Final[float] = 20000.0
PCA9685_TICKS_PER_CYCLE: Final[int] = 4096


# =============================================================================
# Robot Savo Pan-Tilt Hardware Mapping
#
# These are fixed for the validated Robot Savo pan-tilt wiring.
# Freenove logical servo ports are not the same as raw PCA9685 channels.
# =============================================================================
PAN_LOGICAL_CHANNEL_DEFAULT: Final[str] = "7"
TILT_LOGICAL_CHANNEL_DEFAULT: Final[str] = "6"

FREENOVE_LOGICAL_TO_PCA9685_CHANNEL: Final[Mapping[str, int]] = {
    "0": 8,
    "1": 9,
    "2": 10,
    "3": 11,
    "4": 12,
    "5": 13,
    "6": 14,
    "7": 15,
}

PAN_PCA9685_CHANNEL_DEFAULT: Final[int] = FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[
    PAN_LOGICAL_CHANNEL_DEFAULT
]
TILT_PCA9685_CHANNEL_DEFAULT: Final[int] = FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[
    TILT_LOGICAL_CHANNEL_DEFAULT
]

ALLOW_CHANNEL_OVERRIDE_DEFAULT: Final[bool] = False


# =============================================================================
# Pan-Tilt Calibration Defaults
# =============================================================================
PAN_MIN_DEG_DEFAULT: Final[int] = 0
PAN_CENTER_DEG_DEFAULT: Final[int] = 72
PAN_MAX_DEG_DEFAULT: Final[int] = 170

TILT_MIN_DEG_DEFAULT: Final[int] = 45
TILT_CENTER_DEG_DEFAULT: Final[int] = 55
TILT_MAX_DEG_DEFAULT: Final[int] = 130

MANUAL_STEP_DEG_DEFAULT: Final[int] = 2
SERVO_ANGLE_MIN_DEG: Final[int] = 0
SERVO_ANGLE_MAX_DEG: Final[int] = 180

SERVO_ERROR_DEFAULT_DEG: Final[int] = 10


# =============================================================================
# Scan Defaults
# =============================================================================
SCAN_PROFILE_DEFAULT: Final[str] = "semantic_scan"

SCAN_PAN_MIN_DEG_DEFAULT: Final[int] = PAN_MIN_DEG_DEFAULT
SCAN_PAN_CENTER_DEG_DEFAULT: Final[int] = PAN_CENTER_DEG_DEFAULT
SCAN_PAN_MAX_DEG_DEFAULT: Final[int] = PAN_MAX_DEG_DEFAULT

SCAN_TILT_MIN_DEG_DEFAULT: Final[int] = TILT_MIN_DEG_DEFAULT
SCAN_TILT_MAX_DEG_DEFAULT: Final[int] = TILT_MAX_DEG_DEFAULT

SCAN_PAN_STEP_DEG_DEFAULT: Final[int] = 2
SCAN_TILT_STEP_DEG_DEFAULT: Final[int] = 2
SCAN_STEP_DELAY_S_DEFAULT: Final[float] = 0.12

SCAN_PAN_TARGETS_DEFAULT: Final[Tuple[int, int, int, int]] = (
    PAN_CENTER_DEG_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
)

SCAN_TILT_SWEEP_PAN_TARGETS: Final[Tuple[int, ...]] = (
    PAN_CENTER_DEG_DEFAULT,
)


# =============================================================================
# Camera Defaults
#
# GStreamer libcamerasrc is the only validated Pi Camera 2 NoIR backend on the
# current Ubuntu setup. OpenCV VideoCapture is not validated yet.
# =============================================================================
CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC: Final[str] = "gstreamer_libcamerasrc"
CAMERA_BACKEND_OPENCV_UNVALIDATED: Final[str] = "opencv_videocapture_unvalidated"

CAMERA_BACKEND_DEFAULT: Final[str] = CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC

CAMERA_WIDTH_DEFAULT: Final[int] = 640
CAMERA_HEIGHT_DEFAULT: Final[int] = 480
CAMERA_FPS_DEFAULT: Final[int] = 30
CAMERA_FORMAT_DEFAULT: Final[str] = "I420"

CAMERA_UDP_HOST_DEFAULT: Final[str] = ""
CAMERA_UDP_PORT_DEFAULT: Final[int] = 5000
CAMERA_BITRATE_KBPS_DEFAULT: Final[int] = 2000
CAMERA_GST_VERBOSE_DEFAULT: Final[bool] = False

CAMERA_GST_SOURCE_ELEMENT: Final[str] = "libcamerasrc"
CAMERA_GST_ENCODER_ELEMENT: Final[str] = "x264enc"
CAMERA_GST_PAYLOAD_ELEMENT: Final[str] = "rtph264pay"
CAMERA_GST_SINK_ELEMENT: Final[str] = "udpsink"


# =============================================================================
# AprilTag / Semantic Confirmation Defaults
# =============================================================================
APRILTAG_FAMILY_DEFAULT: Final[str] = "tag36h11"
APRILTAG_MIN_STABLE_FRAMES_DEFAULT: Final[int] = 5
APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT: Final[float] = 3.0

SEMANTIC_CONFIRMATION_SOURCE: Final[str] = "savo_head_apriltag"
SEMANTIC_CONFIRMATION_TYPE_SUMMON_POINT: Final[str] = "summon_point"
SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION: Final[str] = "known_location"


# =============================================================================
# QoS / Timing Defaults
# =============================================================================
QOS_DEPTH_DEFAULT: Final[int] = 10
QOS_DEPTH_SENSOR_DEFAULT: Final[int] = 10
QOS_CMD_RELIABILITY_DEFAULT: Final[str] = "reliable"
QOS_SENSOR_RELIABILITY_DEFAULT: Final[str] = "best_effort"

HEAD_CONTROL_HZ_DEFAULT: Final[float] = 30.0
HEAD_STATUS_HZ_DEFAULT: Final[float] = 2.0
HEAD_TF_HZ_DEFAULT: Final[float] = 30.0
HEAD_WATCHDOG_TIMEOUT_S_DEFAULT: Final[float] = 0.50


# =============================================================================
# CLI / Diagnostic Defaults
# =============================================================================
CLI_KEY_TIMEOUT_S_DEFAULT: Final[float] = 0.02
CLI_STATUS_TIMEOUT_S_DEFAULT: Final[float] = 2.0
CLI_CENTER_ON_EXIT_DEFAULT: Final[bool] = True


# =============================================================================
# Status / Diagnostics Labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_STALE: Final[str] = "STALE"
STATUS_DISABLED: Final[str] = "DISABLED"
STATUS_DRYRUN: Final[str] = "DRYRUN"
STATUS_ERROR: Final[str] = "ERROR"

WATCHDOG_NAME_HEAD_COMMAND: Final[str] = "head_command_watchdog"


# =============================================================================
# Structured defaults
# =============================================================================
@dataclass(frozen=True)
class HeadDefaults:
    node_name: str = NODE_NAME_HEAD_CONTROLLER
    robot_name: str = ROBOT_NAME

    pan_tilt_cmd_topic: str = TOPIC_PAN_TILT_CMD
    pan_tilt_state_topic: str = TOPIC_PAN_TILT_STATE
    scan_cmd_topic: str = TOPIC_SCAN_CMD
    scan_state_topic: str = TOPIC_SCAN_STATE
    status_topic: str = TOPIC_STATUS
    dashboard_text_topic: str = TOPIC_DASHBOARD_TEXT
    image_raw_topic: str = TOPIC_IMAGE_RAW
    camera_info_topic: str = TOPIC_CAMERA_INFO
    camera_status_topic: str = TOPIC_CAMERA_STATUS
    apriltag_detections_topic: str = TOPIC_APRILTAG_DETECTIONS
    semantic_confirmations_topic: str = TOPIC_SEMANTIC_CONFIRMATIONS

    base_frame: str = FRAME_BASE
    pan_frame: str = FRAME_PANTILT_PAN
    tilt_frame: str = FRAME_PANTILT_TILT
    camera_frame: str = FRAME_PI_CAMERA
    camera_optical_frame: str = FRAME_PI_CAMERA_OPTICAL

    backend: str = HEAD_BACKEND_DEFAULT
    hardware_profile: str = HEAD_HARDWARE_PROFILE_DEFAULT
    i2c_bus: int = I2C_BUS_DEFAULT
    pca9685_addr: int = PCA9685_ADDR_DEFAULT
    pwm_freq_hz: float = PCA9685_PWM_FREQ_HZ_DEFAULT

    pan_logical_channel: str = PAN_LOGICAL_CHANNEL_DEFAULT
    tilt_logical_channel: str = TILT_LOGICAL_CHANNEL_DEFAULT
    pan_pca9685_channel: int = PAN_PCA9685_CHANNEL_DEFAULT
    tilt_pca9685_channel: int = TILT_PCA9685_CHANNEL_DEFAULT
    allow_channel_override: bool = ALLOW_CHANNEL_OVERRIDE_DEFAULT

    pan_min_deg: int = PAN_MIN_DEG_DEFAULT
    pan_center_deg: int = PAN_CENTER_DEG_DEFAULT
    pan_max_deg: int = PAN_MAX_DEG_DEFAULT

    tilt_min_deg: int = TILT_MIN_DEG_DEFAULT
    tilt_center_deg: int = TILT_CENTER_DEG_DEFAULT
    tilt_max_deg: int = TILT_MAX_DEG_DEFAULT

    manual_step_deg: int = MANUAL_STEP_DEG_DEFAULT

    scan_profile: str = SCAN_PROFILE_DEFAULT
    scan_pan_step_deg: int = SCAN_PAN_STEP_DEG_DEFAULT
    scan_tilt_step_deg: int = SCAN_TILT_STEP_DEG_DEFAULT
    scan_step_delay_s: float = SCAN_STEP_DELAY_S_DEFAULT

    camera_backend: str = CAMERA_BACKEND_DEFAULT
    camera_width: int = CAMERA_WIDTH_DEFAULT
    camera_height: int = CAMERA_HEIGHT_DEFAULT
    camera_fps: int = CAMERA_FPS_DEFAULT
    camera_format: str = CAMERA_FORMAT_DEFAULT
    camera_udp_host: str = CAMERA_UDP_HOST_DEFAULT
    camera_udp_port: int = CAMERA_UDP_PORT_DEFAULT
    camera_bitrate_kbps: int = CAMERA_BITRATE_KBPS_DEFAULT

    apriltag_family: str = APRILTAG_FAMILY_DEFAULT
    apriltag_min_stable_frames: int = APRILTAG_MIN_STABLE_FRAMES_DEFAULT
    apriltag_max_detection_distance_m: float = APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT

    control_hz: float = HEAD_CONTROL_HZ_DEFAULT
    status_hz: float = HEAD_STATUS_HZ_DEFAULT
    tf_hz: float = HEAD_TF_HZ_DEFAULT
    watchdog_timeout_s: float = HEAD_WATCHDOG_TIMEOUT_S_DEFAULT

    def to_dict(self) -> dict:
        return {
            "node_name": self.node_name,
            "robot_name": self.robot_name,
            "topics": {
                "pan_tilt_cmd": self.pan_tilt_cmd_topic,
                "pan_tilt_state": self.pan_tilt_state_topic,
                "scan_cmd": self.scan_cmd_topic,
                "scan_state": self.scan_state_topic,
                "status": self.status_topic,
                "dashboard_text": self.dashboard_text_topic,
                "image_raw": self.image_raw_topic,
                "camera_info": self.camera_info_topic,
                "camera_status": self.camera_status_topic,
                "apriltag_detections": self.apriltag_detections_topic,
                "semantic_confirmations": self.semantic_confirmations_topic,
            },
            "frames": {
                "base": self.base_frame,
                "pan": self.pan_frame,
                "tilt": self.tilt_frame,
                "camera": self.camera_frame,
                "camera_optical": self.camera_optical_frame,
            },
            "hardware": {
                "backend": self.backend,
                "hardware_profile": self.hardware_profile,
                "i2c_bus": self.i2c_bus,
                "pca9685_addr": f"0x{self.pca9685_addr:02X}",
                "pwm_freq_hz": self.pwm_freq_hz,
                "pan_logical_channel": self.pan_logical_channel,
                "tilt_logical_channel": self.tilt_logical_channel,
                "pan_pca9685_channel": self.pan_pca9685_channel,
                "tilt_pca9685_channel": self.tilt_pca9685_channel,
                "allow_channel_override": self.allow_channel_override,
            },
            "calibration": {
                "pan_min_deg": self.pan_min_deg,
                "pan_center_deg": self.pan_center_deg,
                "pan_max_deg": self.pan_max_deg,
                "tilt_min_deg": self.tilt_min_deg,
                "tilt_center_deg": self.tilt_center_deg,
                "tilt_max_deg": self.tilt_max_deg,
                "manual_step_deg": self.manual_step_deg,
            },
            "scan": {
                "profile": self.scan_profile,
                "pan_targets": list(SCAN_PAN_TARGETS_DEFAULT),
                "tilt_sweep_pan_targets": list(SCAN_TILT_SWEEP_PAN_TARGETS),
                "pan_step_deg": self.scan_pan_step_deg,
                "tilt_step_deg": self.scan_tilt_step_deg,
                "step_delay_s": self.scan_step_delay_s,
            },
            "camera": {
                "backend": self.camera_backend,
                "width": self.camera_width,
                "height": self.camera_height,
                "fps": self.camera_fps,
                "format": self.camera_format,
                "udp_host": self.camera_udp_host,
                "udp_port": self.camera_udp_port,
                "bitrate_kbps": self.camera_bitrate_kbps,
            },
            "apriltag": {
                "family": self.apriltag_family,
                "min_stable_frames": self.apriltag_min_stable_frames,
                "max_detection_distance_m": self.apriltag_max_detection_distance_m,
                "semantic_source": SEMANTIC_CONFIRMATION_SOURCE,
            },
            "rates_timeouts": {
                "control_hz": self.control_hz,
                "status_hz": self.status_hz,
                "tf_hz": self.tf_hz,
                "watchdog_timeout_s": self.watchdog_timeout_s,
            },
        }


DEFAULTS: Final[HeadDefaults] = HeadDefaults()


# =============================================================================
# Helpers
# =============================================================================
def get_head_defaults() -> HeadDefaults:
    """Return immutable structured defaults for Robot Savo head components."""
    return DEFAULTS


def clamp_angle_deg(value: int, low: int = SERVO_ANGLE_MIN_DEG, high: int = SERVO_ANGLE_MAX_DEG) -> int:
    """Clamp a servo angle to a safe integer degree range."""
    return max(int(low), min(int(high), int(value)))


def clamp_pulse_us(value: float) -> float:
    """Clamp a servo pulse width to the configured PCA9685 servo envelope."""
    v = float(value)
    if v > PCA9685_PULSE_MAX_US:
        return PCA9685_PULSE_MAX_US
    if v < PCA9685_PULSE_MIN_US:
        return PCA9685_PULSE_MIN_US
    return v


def logical_channel_to_pca9685_channel(channel: str) -> int:
    """Convert a Freenove logical servo channel to the raw PCA9685 channel."""
    key = str(channel)
    if key not in FREENOVE_LOGICAL_TO_PCA9685_CHANNEL:
        valid = ", ".join(sorted(FREENOVE_LOGICAL_TO_PCA9685_CHANNEL))
        raise ValueError(f"Invalid logical servo channel {channel!r}. Valid: {valid}")
    return FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[key]


def is_valid_camera_backend(backend: str) -> bool:
    """Return whether the camera backend name is known by this package."""
    return backend in {
        CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC,
        CAMERA_BACKEND_OPENCV_UNVALIDATED,
    }


__all__ = [
    # Identity
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "NODE_NAME_HEAD_CONTROLLER",
    "NODE_NAME_HEAD_SCAN",
    "NODE_NAME_HEAD_TF",
    "NODE_NAME_HEAD_STATUS",
    "NODE_NAME_APRILTAG_CONFIRM",
    # Topics
    "TOPIC_PAN_TILT_CMD",
    "TOPIC_PAN_TILT_STATE",
    "TOPIC_SCAN_CMD",
    "TOPIC_SCAN_STATE",
    "TOPIC_STATUS",
    "TOPIC_DASHBOARD_TEXT",
    "TOPIC_IMAGE_RAW",
    "TOPIC_CAMERA_INFO",
    "TOPIC_CAMERA_STATUS",
    "TOPIC_APRILTAG_DETECTIONS",
    "TOPIC_SEMANTIC_CONFIRMATIONS",
    # Frames
    "FRAME_BASE",
    "FRAME_PANTILT_PAN",
    "FRAME_PANTILT_TILT",
    "FRAME_PI_CAMERA",
    "FRAME_PI_CAMERA_OPTICAL",
    # Hardware
    "HEAD_BACKEND_DEFAULT",
    "HEAD_HARDWARE_PROFILE_DEFAULT",
    "I2C_BUS_DEFAULT",
    "PCA9685_ADDR_DEFAULT",
    "PCA9685_PWM_FREQ_HZ_DEFAULT",
    "PCA9685_PULSE_MIN_US",
    "PCA9685_PULSE_MAX_US",
    "PCA9685_SERVO_PERIOD_US",
    "PCA9685_TICKS_PER_CYCLE",
    "PAN_LOGICAL_CHANNEL_DEFAULT",
    "TILT_LOGICAL_CHANNEL_DEFAULT",
    "FREENOVE_LOGICAL_TO_PCA9685_CHANNEL",
    "PAN_PCA9685_CHANNEL_DEFAULT",
    "TILT_PCA9685_CHANNEL_DEFAULT",
    "ALLOW_CHANNEL_OVERRIDE_DEFAULT",
    # Calibration
    "PAN_MIN_DEG_DEFAULT",
    "PAN_CENTER_DEG_DEFAULT",
    "PAN_MAX_DEG_DEFAULT",
    "TILT_MIN_DEG_DEFAULT",
    "TILT_CENTER_DEG_DEFAULT",
    "TILT_MAX_DEG_DEFAULT",
    "MANUAL_STEP_DEG_DEFAULT",
    "SERVO_ANGLE_MIN_DEG",
    "SERVO_ANGLE_MAX_DEG",
    "SERVO_ERROR_DEFAULT_DEG",
    # Scan
    "SCAN_PROFILE_DEFAULT",
    "SCAN_PAN_MIN_DEG_DEFAULT",
    "SCAN_PAN_CENTER_DEG_DEFAULT",
    "SCAN_PAN_MAX_DEG_DEFAULT",
    "SCAN_TILT_MIN_DEG_DEFAULT",
    "SCAN_TILT_MAX_DEG_DEFAULT",
    "SCAN_PAN_STEP_DEG_DEFAULT",
    "SCAN_TILT_STEP_DEG_DEFAULT",
    "SCAN_STEP_DELAY_S_DEFAULT",
    "SCAN_PAN_TARGETS_DEFAULT",
    "SCAN_TILT_SWEEP_PAN_TARGETS",
    # Camera
    "CAMERA_BACKEND_GSTREAMER_LIBCAMERASRC",
    "CAMERA_BACKEND_OPENCV_UNVALIDATED",
    "CAMERA_BACKEND_DEFAULT",
    "CAMERA_WIDTH_DEFAULT",
    "CAMERA_HEIGHT_DEFAULT",
    "CAMERA_FPS_DEFAULT",
    "CAMERA_FORMAT_DEFAULT",
    "CAMERA_UDP_HOST_DEFAULT",
    "CAMERA_UDP_PORT_DEFAULT",
    "CAMERA_BITRATE_KBPS_DEFAULT",
    "CAMERA_GST_VERBOSE_DEFAULT",
    "CAMERA_GST_SOURCE_ELEMENT",
    "CAMERA_GST_ENCODER_ELEMENT",
    "CAMERA_GST_PAYLOAD_ELEMENT",
    "CAMERA_GST_SINK_ELEMENT",
    # AprilTag / semantic
    "APRILTAG_FAMILY_DEFAULT",
    "APRILTAG_MIN_STABLE_FRAMES_DEFAULT",
    "APRILTAG_MAX_DETECTION_DISTANCE_M_DEFAULT",
    "SEMANTIC_CONFIRMATION_SOURCE",
    "SEMANTIC_CONFIRMATION_TYPE_SUMMON_POINT",
    "SEMANTIC_CONFIRMATION_TYPE_KNOWN_LOCATION",
    # QoS / timing
    "QOS_DEPTH_DEFAULT",
    "QOS_DEPTH_SENSOR_DEFAULT",
    "QOS_CMD_RELIABILITY_DEFAULT",
    "QOS_SENSOR_RELIABILITY_DEFAULT",
    "HEAD_CONTROL_HZ_DEFAULT",
    "HEAD_STATUS_HZ_DEFAULT",
    "HEAD_TF_HZ_DEFAULT",
    "HEAD_WATCHDOG_TIMEOUT_S_DEFAULT",
    # CLI / status
    "CLI_KEY_TIMEOUT_S_DEFAULT",
    "CLI_STATUS_TIMEOUT_S_DEFAULT",
    "CLI_CENTER_ON_EXIT_DEFAULT",
    "STATUS_OK",
    "STATUS_STALE",
    "STATUS_DISABLED",
    "STATUS_DRYRUN",
    "STATUS_ERROR",
    "WATCHDOG_NAME_HEAD_COMMAND",
    # Structured defaults + helpers
    "HeadDefaults",
    "DEFAULTS",
    "get_head_defaults",
    "clamp_angle_deg",
    "clamp_pulse_us",
    "logical_channel_to_pca9685_channel",
    "is_valid_camera_backend",
]
