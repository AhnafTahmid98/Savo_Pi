#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Constants for Robot Savo localization. No ROS imports — safe to import anywhere."""

from __future__ import annotations

from typing import Final, Tuple


# =============================================================================
# Package identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_localization"
ROBOT_NAME: Final[str] = "Robot Savo"
SUPPORTED_ROS_DISTRO: Final[str] = "jazzy"


# =============================================================================
# Frames
# =============================================================================
FRAME_ODOM: Final[str] = "odom"
FRAME_BASE_LINK: Final[str] = "base_link"
FRAME_IMU: Final[str] = "imu_link"
FRAME_WHEEL_ODOM: Final[str] = "wheel_odom"


# =============================================================================
# ROS topic contract
# =============================================================================
DEFAULT_IMU_TOPIC: Final[str] = "/imu/data"
DEFAULT_WHEEL_ODOM_TOPIC: Final[str] = "/wheel/odom"
DEFAULT_FILTERED_ODOM_TOPIC: Final[str] = "/odometry/filtered"

DEFAULT_IMU_STATE_TOPIC: Final[str] = "/savo_localization/imu_state"
DEFAULT_ENCODER_STATE_TOPIC: Final[str] = "/savo_localization/encoder_state"
DEFAULT_WHEEL_ODOM_STATE_TOPIC: Final[str] = "/savo_localization/wheel_odom_state"
DEFAULT_EKF_HEALTH_TOPIC: Final[str] = "/savo_localization/ekf_health"
DEFAULT_STATE_SUMMARY_TOPIC: Final[str] = "/savo_localization/state_summary"


# =============================================================================
# Hardware identity
# =============================================================================
IMU_MODEL_BNO055: Final[str] = "bno055"
ENCODER_MODEL_QUADRATURE: Final[str] = "quadrature_gpio"
ODOM_MODEL_MECANUM_4ENC: Final[str] = "mecanum_4enc"


# =============================================================================
# BNO055 defaults
# =============================================================================
BNO055_DEFAULT_I2C_BUS: Final[int] = 1
BNO055_DEFAULT_ADDRESS: Final[int] = 0x28
BNO055_CHIP_ID: Final[int] = 0xA0

BNO055_MODE_IMU: Final[str] = "imu"
BNO055_MODE_NDOF: Final[str] = "ndof"
BNO055_DEFAULT_MODE: Final[str] = BNO055_MODE_NDOF

DEFAULT_IMU_RATE_HZ: Final[float] = 25.0
DEFAULT_IMU_SAMPLE_COUNT: Final[int] = 200

GRAVITY_MPS2: Final[float] = 9.80665

IMU_HEALTH_GRADE_A: Final[str] = "A"
IMU_HEALTH_GRADE_B: Final[str] = "B"
IMU_HEALTH_GRADE_C: Final[str] = "C"

DEFAULT_IMU_GYRO_RMS_MOVING_DPS: Final[float] = 2.0
DEFAULT_IMU_ACCEL_STD_MOVING_MPS2: Final[float] = 0.15
DEFAULT_IMU_MOTION_WINDOW_SAMPLES: Final[int] = 20


# =============================================================================
# Four-wheel encoder GPIO defaults
# =============================================================================
WHEEL_FL: Final[str] = "FL"
WHEEL_FR: Final[str] = "FR"
WHEEL_RL: Final[str] = "RL"
WHEEL_RR: Final[str] = "RR"

WHEEL_ORDER: Final[Tuple[str, str, str, str]] = (
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
)

DEFAULT_FL_A_GPIO: Final[int] = 25
DEFAULT_FL_B_GPIO: Final[int] = 13

DEFAULT_FR_A_GPIO: Final[int] = 20
DEFAULT_FR_B_GPIO: Final[int] = 21

DEFAULT_RL_A_GPIO: Final[int] = 23
DEFAULT_RL_B_GPIO: Final[int] = 24

DEFAULT_RR_A_GPIO: Final[int] = 12
DEFAULT_RR_B_GPIO: Final[int] = 26

DEFAULT_ENCODER_GPIO_MAP: Final[dict[str, tuple[int, int]]] = {
    WHEEL_FL: (DEFAULT_FL_A_GPIO, DEFAULT_FL_B_GPIO),
    WHEEL_FR: (DEFAULT_FR_A_GPIO, DEFAULT_FR_B_GPIO),
    WHEEL_RL: (DEFAULT_RL_A_GPIO, DEFAULT_RL_B_GPIO),
    WHEEL_RR: (DEFAULT_RR_A_GPIO, DEFAULT_RR_B_GPIO),
}

DEFAULT_ENCODER_INVERT_FL: Final[bool] = True
DEFAULT_ENCODER_INVERT_FR: Final[bool] = True
DEFAULT_ENCODER_INVERT_RL: Final[bool] = True
DEFAULT_ENCODER_INVERT_RR: Final[bool] = True

DEFAULT_ENCODER_INVERT_MAP: Final[dict[str, bool]] = {
    WHEEL_FL: DEFAULT_ENCODER_INVERT_FL,
    WHEEL_FR: DEFAULT_ENCODER_INVERT_FR,
    WHEEL_RL: DEFAULT_ENCODER_INVERT_RL,
    WHEEL_RR: DEFAULT_ENCODER_INVERT_RR,
}

DEFAULT_ENCODER_POLL_S: Final[float] = 0.001
DEFAULT_ENCODER_DEBOUNCE_S: Final[float] = 0.0003
DEFAULT_ENCODER_REPORT_INTERVAL_S: Final[float] = 0.5

DEFAULT_USE_INTERNAL_PULLUP: Final[bool] = False
DEFAULT_USE_HW_DEBOUNCE: Final[bool] = True


# =============================================================================
# Wheel and odometry defaults
# =============================================================================
DEFAULT_WHEEL_DIAMETER_M: Final[float] = 0.065
DEFAULT_ENCODER_CPR: Final[int] = 20
DEFAULT_ENCODER_DECODING: Final[int] = 4
DEFAULT_GEAR_RATIO: Final[float] = 1.0

DEFAULT_WHEELBASE_M: Final[float] = 0.165
DEFAULT_TRACK_M: Final[float] = 0.165

DEFAULT_WHEEL_ODOM_RATE_HZ: Final[float] = 30.0
DEFAULT_WHEEL_ODOM_TIMEOUT_S: Final[float] = 0.5
DEFAULT_ODOM_COVARIANCE_SCALE: Final[float] = 1.0


# =============================================================================
# EKF defaults
# =============================================================================
DEFAULT_EKF_NODE_NAME: Final[str] = "ekf_filter_node"
DEFAULT_EKF_RATE_HZ: Final[float] = 30.0
DEFAULT_EKF_SENSOR_TIMEOUT_S: Final[float] = 0.2

DEFAULT_PUBLISH_TF: Final[bool] = True
DEFAULT_TWO_D_MODE: Final[bool] = True


# =============================================================================
# Health/status values
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_WARN: Final[str] = "WARN"
STATUS_ERROR: Final[str] = "ERROR"
STATUS_STALE: Final[str] = "STALE"
STATUS_OFFLINE: Final[str] = "OFFLINE"
STATUS_UNKNOWN: Final[str] = "UNKNOWN"

HEALTH_LEVEL_OK: Final[int] = 0
HEALTH_LEVEL_WARN: Final[int] = 1
HEALTH_LEVEL_ERROR: Final[int] = 2
HEALTH_LEVEL_STALE: Final[int] = 3


# =============================================================================
# Diagnostic thresholds
# =============================================================================
DEFAULT_MIN_ENCODER_RATE_CPS: Final[float] = 0.0
DEFAULT_MAX_ILLEGAL_TRANSITIONS: Final[int] = 5

DEFAULT_MIN_VALID_WHEEL_COUNT: Final[int] = 4
DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS: Final[float] = 1.0
DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S: Final[float] = 3.0

DEFAULT_IMU_TEMP_MIN_C: Final[float] = 0.0
DEFAULT_IMU_TEMP_MAX_C: Final[float] = 60.0
DEFAULT_IMU_TEMP_WARN_LOW_C: Final[float] = 5.0
DEFAULT_IMU_TEMP_WARN_HIGH_C: Final[float] = 55.0

DEFAULT_GRAVITY_ERROR_WARN_MPS2: Final[float] = 0.4
DEFAULT_GRAVITY_ERROR_ERROR_MPS2: Final[float] = 0.8

DEFAULT_GYRO_Z_BIAS_WARN_DPS: Final[float] = 0.8
DEFAULT_GYRO_Z_BIAS_ERROR_DPS: Final[float] = 1.5


# =============================================================================
# Logging/reporting
# =============================================================================
DEFAULT_JSON_SORT_KEYS: Final[bool] = True
DEFAULT_JSON_COMPACT: Final[bool] = True