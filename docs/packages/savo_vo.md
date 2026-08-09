# savo_vo

## Purpose

Edge RGB-D visual odometry, republishing, health, quality, covariance, and diagnostics.

## Deployment

Edge only; production bringup enables VO by default, but Core EKF fusion defaults disabled until validation.

## Responsibilities

Synchronize color/depth/info, estimate RGB-D motion, reject jumps/stale input, publish raw/canonical odometry with covariance and tracking quality, and expose health/diagnostics.

## Non-responsibilities and authority boundaries

Does not own D435, `map -> odom`, Core `odom -> base`, navigation admission, or motion; does not replace wheel odometry/IMU.

## Package structure

C++ production nodes/core algorithms, Python retained implementation/diagnostics, YAML profiles, launch files, tests.

## Runtime components

### Production nodes

`rgbd_odometry_node`, `vo_republisher_node`, `vo_health_node`, `vo_diagnostics_node`. Launch `implementation` can select retained Python explicitly; C++ is authoritative.

## Runtime data flow

`D435 color+depth+info -> RGB-D odometry -> /vo/odom/raw -> republisher -> /vo/odom -> optional Core EKF`; status/quality feed health/diagnostics.

## ROS interfaces

### Published topics

`/vo/odom/raw`, `/vo/odom` (`nav_msgs/msg/Odometry`), `/vo/status`, `/vo/health` (String), `/vo/tracking_quality` (Float32), `/diagnostics`.

### Subscribed topics

Configured RealSense aligned color/depth and CameraInfo; republisher/health/diagnostics consume raw/canonical odom/status/health.

### Services

No public service.

### Actions

No action.

## TF ownership

No production map/base TF authority. Frames include `vo_odom`/`vo_camera_link`; Core fusion owns robot odom TF.

## Parameters and configuration

Input delay limits 0.15 s; minimum/good/maximum features `80/300/800`; maximum translation/rotation jump `0.30 m/0.35 rad`; covariance, QoS, frames, freshness, and health thresholds in YAML.

## Launch files

`vo_bringup` composes all four nodes; per-node odometry, republisher, health, diagnostics launches support isolation.

## Persistent state and runtime files

None.

## Hardware ownership

None directly; consumes `savo_realsense` streams.

## Dependencies

### Internal Robot Savo dependencies

RealSense inputs; optional localization fusion; bringup/supervisor/UI/observer health.

### External ROS/system dependencies

OpenCV/RGB-D algorithms, cv_bridge, sensor/nav/diagnostic messages.

## Safety behavior

Stale/misaligned/insufficient-feature or jump-rejected estimates become unhealthy and must not be fused. Fusion is explicitly gated and loss cannot block STOP.

## Failure and degraded behavior

Dropout marks health stale; localization continues without VO under its configured fusion policy rather than trusting last pose.

## Startup and shutdown behavior

Waits for synchronized camera data/calibration and initializes origin; cleanly stops publication on exit.

## Build

`bash deploy/edge/build_edge.sh --clean --test`.

## Run

`ros2 launch savo_vo vo_bringup.launch.py implementation:=cpp`.

## Validation and testing

Tests cover layout/migration, frames/topics/params, covariance, geometry, sync, motion/quality, launch profiles, and status.

## Current validation status

Implemented/source-tested; stable live output, covariance, timestamps, dropout/recovery, agreement, then guarded EKF fusion require hardware/integration validation.

## Known limitations and remaining validation

Core deployment defaults `localization_use_vo=false`; physical calibration and compute-rate performance are not established for current source.

## Change-control considerations

Frames, timestamp sync, jump gates, covariance, estimator implementation, or enabling fusion require localization regression.

## Related documentation

- [Implementation README](../../savo_ws/src/edge/savo_vo/README.md)
- [VO test plan](../testing/vo_test_plan.md)
- [Localization architecture](../architecture/localization_architecture.md)
- [Ownership matrix](package_ownership_matrix.md)
