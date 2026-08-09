# savo_realsense

## Purpose

Edge ownership of Intel RealSense D435 acquisition, stream profiles, monitoring, health, and front-depth extraction.

## Deployment

Edge only. Production bringup starts RealSense by default; point-cloud use remains profile/gate dependent.

## Responsibilities

Configure color/depth/alignment/sync/pointcloud, monitor stream rate/freshness, publish camera health/diagnostics, and derive a robust central front-depth value.

## Non-responsibilities and authority boundaries

Does not own safety fusion, filtered obstacle cloud, VO estimation, fixed camera TF, navigation admission, or motion.

## Package structure

C++ monitoring/depth nodes, external `realsense2_camera`, launch/profile YAML, Python diagnostic utilities, tests.

## Runtime components

### Nodes

External `realsense2_camera_node` owns device streams. `camera_topic_monitor_node` and `camera_health_node` check freshness/rates; `depth_front_min_node` computes a central-ROI percentile depth. C++ is production.

## Runtime data flow

`D435 -> aligned color/depth/info/points -> VO, perception, health`; depth image also produces `/depth/min_front_m`.

## ROS interfaces

### Published topics

RealSense topics under `/camera/camera/...`, including color/depth images, CameraInfo, aligned depth, and configured point cloud; `/savo_realsense/status`, `/diagnostics`; `/depth/min_front_m` (`Float32`).

### Subscribed topics

Monitor/health consume color/depth/info/pointcloud; front-depth consumes configured aligned depth image.

### Services

Device services are those of `realsense2_camera`; this package defines no new public service.

### Actions

No action.

## TF ownership

RealSense `publish_tf=false`; `savo_description` exclusively owns fixed camera frames.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| serial | `801212070967` | Repository-bound D435 identity; verify deployed unit |
| depth/color | `848x480x30`, `640x480x30` | Streams |
| alignment/sync | `true/true` | RGB-D pairing |
| pointcloud | enabled in D435 profile | Optional consumers |
| stale timeout | `0.75 s` | Health |
| front ROI | x `0.35–0.65`, y `0.35–0.75` | Depth region |
| depth percentile/range | `10%`, `0.02–3.0 m` | Robust result |

## Launch files

`realsense_bringup`, `realsense_minimal`, `realsense_vo`, `realsense_pointcloud`, and diagnostics profiles.

## Persistent state and runtime files

None; device calibration/firmware is external. Serial is source configuration.

## Hardware ownership

One Edge-connected D435 and its USB bandwidth/device process.

## Dependencies

### Internal Robot Savo dependencies

Description TF, VO image consumer, perception depth/cloud consumer, bringup/supervisor/UI health.

### External ROS/system dependencies

`realsense2_camera`, librealsense, USB3, sensor/diagnostic messages.

## Safety behavior

Stale/missing/low-rate streams mark unhealthy. D435 loss cannot disable STOP; optional voxel/cloud navigation must close its readiness gate.

## Failure and degraded behavior

Driver/device loss removes streams; monitors report stale. Reconnection uses driver timeout/reset policy and must not publish false health.

## Startup and shutdown behavior

Starts/reset policy per profile (`initial_reset=false`, reconnect timeout 6 s); cleanly releases device.

## Build

`bash deploy/edge/build_edge.sh --clean --test`.

## Run

`ros2 launch savo_realsense realsense_bringup.launch.py`.

## Validation and testing

Tests cover topics/frames, serial binding, stream status/timing, depth math, pointcloud profile, diagnostics, and C++ defaults.

## Current validation status

Implemented with retained PC evidence; live USB3, serial, profiles/rates, alignment, restart, depth, thermal/bandwidth validation required.

## Known limitations and remaining validation

Serial must match hardware; firmware is not asserted as source truth. Pointcloud/voxel path remains physically gated.

## Change-control considerations

Serial, streams, alignment, TF policy, QoS, ROI, or pointcloud enablement affect VO/safety/nav integration.

## Related documentation

- [Implementation README](../../savo_ws/src/edge/savo_realsense/README.md)
- [RealSense setup](../setup/realsense_setup.md)
- [Edge architecture](../architecture/savo_edge_architecture.md)
- [Ownership matrix](package_ownership_matrix.md)

