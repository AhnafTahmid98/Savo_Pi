# Robot Savo System Overview

Robot Savo is a ROS 2 Jazzy mobile robot split across a safety-critical Core computer and a compute-heavy Edge computer. This page defines the system boundary; package details live in the [package pages](../packages/package_ownership_matrix.md).

## System boundary

```text
Operator / SavoMind
        |
        | typed requests and observations
        v
Edge: bridge, speech, UI, RealSense, VO
        |
        | ROS 2 over dedicated Ethernet
        v
Core: supervisor, control, safety, localization, mapping, navigation
        |
        v
base driver -> PCA9685 -> motors
```

Core owns every physical motion decision and drivetrain output. Edge owns local SavoMind transport, audio, display, RGB-D acquisition, and visual odometry. The observer workstation is read-only. `savo_msgs`, `savo_description`, and selected shared packages provide contracts used by both hosts.

## Hard authority contracts

The only production drivetrain chain is:

```text
approved source -> savo_control -> /cmd_vel
    -> savo_perception -> /cmd_vel_safe -> savo_base -> motors
```

`savo_supervisor` grants or revokes permission; it does not execute navigation, mapping, or motor commands. Operation owners must retain their own readiness and cancellation checks. `savo_bridge` exposes a finite set of typed, bounded operations. SavoMind, UI, and observer cannot publish directly into the motor path or approve map/location releases.

The system fails closed on stale command input, safety-stop input, invalid bringup combinations, missing required readiness, unverified production-map context, and lost supervisor permission. Core control starts in `STOP`.

## State and data flow

LiDAR, encoders, IMU, near-field range sensors, and power telemetry originate on Core. Edge contributes RealSense streams, optional `/vo/odom`, Edge UPS state, bridge state, and optional speech/UI health. Core fuses wheel odometry and IMU into `/odometry/filtered`; VO fusion is configured off by default. SLAM or AMCL owns `map -> odom`, while the EKF owns `odom -> base_footprint`.

Mapping creates immutable session artifacts, quality reports, review state, and transactional production releases. Navigation accepts only a verified active-map contract. Semantic locations use a Core SQLite registry and operator-controlled release lifecycle.

## Bringup and degraded operation

`savo_bringup` selects host role, robot mode, and profile. Core and Edge publish independent readiness under `/savo_bringup/core/*` and `/savo_bringup/edge/*`. Missing required dependencies progress from a waiting state to `blocked` after the startup timeout. Optional unhealthy dependencies yield `degraded` while readiness remains true.

Production motion requires locked geometry. The `lidar_d435_voxel` profile additionally requires explicit D435 obstacle-cloud validation. Losing optional Edge services may degrade speech, display, or VO; it must not transfer authority away from Core.

## Persistence and operations

Core state is rooted at `/var/lib/robot_savo`; Core logs are rooted at `/var/log/robot_savo`. Edge bridge and speech IPC use `/run/savo_bridge` and `/run/savomind`. See [data storage](data_storage_and_artifacts.md), [network architecture](network_architecture.md), and [diagnostics](diagnostics_and_observability.md).

## Validation status

Source-contract, role, bringup, observer, and aggregate pre-real-test validators exist and pass in the current development checkout. This is not physical authorization. The current geometry profile is provisional, current-target regression is outstanding, D435 voxel use is unvalidated by default, and the hardware gaps in the [calibration register](../hardware/calibration_register.md) remain open.
