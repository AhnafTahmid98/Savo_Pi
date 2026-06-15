# Copyright 2026 Ahnaf Tahmid
from __future__ import annotations

from dataclasses import dataclass

from savo_realsense.models.stream_status import StreamStatus


@dataclass(frozen=True)
class CameraStatus:
    color: StreamStatus
    color_info: StreamStatus
    depth: StreamStatus
    depth_info: StreamStatus
    pointcloud: StreamStatus | None = None
    require_pointcloud: bool = False

    @property
    def color_ok(self) -> bool:
        return self.color.ok

    @property
    def color_info_ok(self) -> bool:
        return self.color_info.ok

    @property
    def depth_ok(self) -> bool:
        return self.depth.ok

    @property
    def depth_info_ok(self) -> bool:
        return self.depth_info.ok

    @property
    def pointcloud_ok(self) -> bool:
        if self.pointcloud is None:
            return not self.require_pointcloud
        return self.pointcloud.ok

    @property
    def any_stream_seen(self) -> bool:
        streams = [self.color, self.color_info, self.depth, self.depth_info]
        if self.pointcloud is not None:
            streams.append(self.pointcloud)
        return any(stream.seen for stream in streams)

    @property
    def ok(self) -> bool:
        return (
            self.color_ok
            and self.color_info_ok
            and self.depth_ok
            and self.depth_info_ok
            and self.pointcloud_ok
        )

    @property
    def message(self) -> str:
        if self.ok:
            return "RealSense streams OK"
        if not self.any_stream_seen:
            return "No RealSense streams detected"

        missing = []
        if not self.color_ok:
            missing.append("color")
        if not self.color_info_ok:
            missing.append("color_info")
        if not self.depth_ok:
            missing.append("depth")
        if not self.depth_info_ok:
            missing.append("depth_info")
        if not self.pointcloud_ok:
            missing.append("pointcloud")

        return "Unhealthy streams: " + ", ".join(missing)
