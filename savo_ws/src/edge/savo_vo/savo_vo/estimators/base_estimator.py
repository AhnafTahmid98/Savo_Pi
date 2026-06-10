"""Base interface for visual odometry estimators."""

from abc import ABC, abstractmethod

from savo_vo.adapters.camera_info_adapter import CameraIntrinsics
from savo_vo.models.vo_sample import VOSample


class BaseVOEstimator(ABC):
    @abstractmethod
    def reset(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def process_frame(
        self,
        timestamp_s: float,
        gray_image: object,
        depth_m: object,
        intrinsics: CameraIntrinsics,
    ) -> VOSample | None:
        raise NotImplementedError