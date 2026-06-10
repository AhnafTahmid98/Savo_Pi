"""Extract camera calibration values from ROS CameraInfo messages."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str

    @property
    def is_valid(self) -> bool:
        return (
            self.width > 0
            and self.height > 0
            and self.fx > 0.0
            and self.fy > 0.0
        )


def camera_info_to_intrinsics(msg: object) -> CameraIntrinsics:
    k = getattr(msg, "k")
    header = getattr(msg, "header")

    return CameraIntrinsics(
        width=int(getattr(msg, "width")),
        height=int(getattr(msg, "height")),
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        frame_id=str(getattr(header, "frame_id")),
    )