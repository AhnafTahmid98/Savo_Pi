"""Odometry quality model for visual odometry output."""

from dataclasses import dataclass
from enum import Enum


class OdometryQualityLevel(str, Enum):
    GOOD = "good"
    OK = "ok"
    POOR = "poor"
    INVALID = "invalid"


@dataclass(frozen=True)
class OdometryQuality:
    level: OdometryQualityLevel
    position_variance: float
    yaw_variance: float
    tracking_quality: float
    message: str = ""

    @property
    def is_valid(self) -> bool:
        return self.level != OdometryQualityLevel.INVALID

    @property
    def is_good(self) -> bool:
        return self.level == OdometryQualityLevel.GOOD

    @property
    def is_usable_for_ekf(self) -> bool:
        return self.level in {
            OdometryQualityLevel.GOOD,
            OdometryQualityLevel.OK,
            OdometryQualityLevel.POOR,
        }

    @classmethod
    def good(
        cls,
        tracking_quality: float,
        position_variance: float = 0.03,
        yaw_variance: float = 0.04,
        message: str = "visual odometry quality is good",
    ) -> "OdometryQuality":
        return cls(
            level=OdometryQualityLevel.GOOD,
            position_variance=position_variance,
            yaw_variance=yaw_variance,
            tracking_quality=tracking_quality,
            message=message,
        )

    @classmethod
    def ok(
        cls,
        tracking_quality: float,
        position_variance: float = 0.08,
        yaw_variance: float = 0.12,
        message: str = "visual odometry quality is acceptable",
    ) -> "OdometryQuality":
        return cls(
            level=OdometryQualityLevel.OK,
            position_variance=position_variance,
            yaw_variance=yaw_variance,
            tracking_quality=tracking_quality,
            message=message,
        )

    @classmethod
    def poor(
        cls,
        tracking_quality: float,
        position_variance: float = 0.25,
        yaw_variance: float = 0.35,
        message: str = "visual odometry quality is poor",
    ) -> "OdometryQuality":
        return cls(
            level=OdometryQualityLevel.POOR,
            position_variance=position_variance,
            yaw_variance=yaw_variance,
            tracking_quality=tracking_quality,
            message=message,
        )

    @classmethod
    def invalid(cls, message: str = "visual odometry quality is invalid") -> "OdometryQuality":
        return cls(
            level=OdometryQualityLevel.INVALID,
            position_variance=999.0,
            yaw_variance=999.0,
            tracking_quality=0.0,
            message=message,
        )