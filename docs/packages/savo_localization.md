# savo_localization

## Purpose

Owns Core BNO055/encoder acquisition, mecanum odometry, EKF fusion, dynamic `odom -> base_footprint`, and localization health.

## Deployment

Core only. Normal bringup starts IMU, wheel odometry, `robot_localization` EKF, and health; VO fusion is opt-in and defaults off.

## Responsibilities

Read IMU and four encoders, estimate wheel odometry, fuse planar motion, publish health, and reject stale/implausible inputs.

## Non-responsibilities and authority boundaries

Does not own `map -> odom`, fixed TF, motion, map selection, or goal admission.

## Package structure

Production C++ drivers/odometry/health; external EKF configured here; Python diagnostic CLIs/support only.

## Runtime components

### Nodes

`imu_node`, `wheel_odom_node`, external `ekf_node`, and `localization_health_node`.

## Runtime data flow

`BNO055 -> /imu/data` and `encoders -> /wheel/odom` (plus optional `/vo/odom`) -> EKF -> `/odometry/filtered` and TF.

## ROS interfaces

### Published topics

`/imu/data` (`Imu`), `/wheel/odom` and `/odometry/filtered` (`Odometry`), `/savo_localization/{imu_state,wheel_odom_state,ekf_state,health,state_summary}` (String), `/diagnostics`.

### Subscribed topics

EKF consumes wheel odom, IMU, optional `/vo/odom`; health consumes production outputs and TF.

### Services

No public service.

### Actions

No action.

## TF ownership

EKF owns dynamic `odom -> base_footprint`; wheel odom keeps `publish_tf: false`. SLAM/AMCL owns `map -> odom`; description owns fixed TF.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| BNO bus/address/mode | `1/0x28/ndof` | IMU |
| IMU/wheel/EKF rates | `25/30/30 Hz` | Timing |
| encoder GPIO FL/FR/RL/RR | `20-21/13-25/23-24/26-12` | Quadrature inputs |
| CPR/decoding | `20/4` | Tick conversion |
| diameter/base/track | `0.065/0.160/0.216 m` | Measured wheel geometry |
| EKF sensor timeout | `0.2 s` | Freshness |
| `use_vo` | `false` | Integration gate |

## Launch files

Production `localization_bringup`; component `imu`, `encoders`, `wheel_odom`, `ekf`; bench/dashboard/diagnostics/dry-run launches are staged.

## Persistent state and runtime files

No database. EKF debug path is `/tmp/...` but debug defaults false.

## Hardware ownership

BNO055 over I2C and four GPIO encoders; no motors.

## Dependencies

### Internal Robot Savo dependencies

Description geometry/frames, optional Edge VO, and consumers in control/mapping/nav/supervisor/head/UI.

### External ROS/system dependencies

`robot_localization`, TF2, liblgpio, Linux I2C, sensor/nav/diagnostic messages.

## Safety behavior

Stale/non-finite/implausible data or missing TF marks unhealthy and must close mapping/navigation readiness. VO cannot become authority unless enabled. STOP remains available.

## Failure and degraded behavior

Sensor loss degrades health; EKF may continue with remaining configured inputs, but readiness decides acceptance. Duplicate TF is prohibited.

## Startup and shutdown behavior

IMU may reset; wheel odom initializes at zero; shutdown releases hardware and stops publication.

## Build

`bash deploy/core/build_core.sh --clean --test`

## Run

`ros2 launch savo_localization localization_bringup.launch.py use_vo:=false`

## Validation and testing

Tests cover encoder/odom math, covariance, kinematics, frames/topics, health, EKF assets, and TF.

## Current validation status

Implemented with PC evidence/earlier baseline; GPIO signs, IMU orientation, EKF/TF, drift, and stale regression required.

## Known limitations and remaining validation

Geometry remains provisional despite synchronized wheel centers: CPR/polarity, loaded radius/slip, covariance, and BNO055 +X/+Y orientation need hardware validation. VO fusion remains gated.

## Change-control considerations

TF authority, GPIO/signs, geometry, covariance, EKF masks, timeouts, or VO enablement require regression.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_localization/README.md)
- [Localization architecture](../architecture/localization_architecture.md)
- [Localization test plan](../testing/localization_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
