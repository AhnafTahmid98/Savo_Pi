"""Internal visual odometry sample model."""

from dataclasses import dataclass

from savo_vo.models.odometry_quality import OdometryQuality
from savo_vo.models.tracking_report import TrackingReport
from savo_vo.models.vo_status import VOStatus


@dataclass(frozen=True)
class VOSample:
    timestamp_s: float
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    yaw_rate_radps: float
    tracking: TrackingReport
    quality: OdometryQuality
    status: VOStatus

    @property
    def is_valid(self) -> bool:
        return (
            self.status.is_usable
            and self.quality.is_valid
            and self.tracking.is_usable
        )

    @property
    def planar_pose(self) -> tuple[float, float, float]:
        return self.x_m, self.y_m, self.yaw_rad

    @property
    def planar_velocity(self) -> tuple[float, float, float]:
        return self.vx_mps, self.vy_mps, self.yaw_rate_radps