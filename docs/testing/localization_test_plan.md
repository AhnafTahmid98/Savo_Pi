# Localization Test Plan

## Objective

Verify four-channel encoder acquisition, mecanum wheel odometry, BNO055 IMU, EKF fusion, covariance/health, and exclusive `odom -> base_footprint` authority. This is local odometry; global `map -> odom` belongs to SLAM/AMCL.

## Scope

GPIO/sign/CPR/scale, straight/strafe/yaw kinematics, IMU orientation/covariance, stale/dropout, `/wheel/odom`, `/odometry/filtered`, TF, drift, reset/restart, and optional VO fusion gate.

## Test ownership

Localization maintainer owns LOC-001–004; hardware/integration stages require motion-test lead and safety operator.

## Safety classification

LOC-001–004 are `STATIC`, `UNIT`, `PC`, or `HARDWARE-NON-ACTUATING`; LOC-005–008 are `HARDWARE-ACTUATING`/`WHEELS-RAISED` then `GUARDED-FLOOR-MOTION`, `INTEGRATION`, `FAULT-INJECTION`, and `RECOVERY`.

## Preconditions

Locked geometry, verified motor/encoder signs, stationary safe-idle, BNO mount/orientation known, time synchronized, connected fixed TF, base/perception plans passed before motion, and VO disabled initially.

## Required hardware

Core Pi, four quadrature encoders, BNO055, rated stands, measured floor course/angle references, E-stop and operators.

## Required software / configuration

Encoder/IMU/wheel odom/EKF/frame/health configs; `robot_localization`; diagnostics; geometry digest; optional VO profile only after VO acceptance.

## Interfaces under test

`/imu/data`, `/wheel/odom`, `/odometry/filtered`, localization state/health/diagnostics and `odom -> base_footprint`. Optional input `/vo/odom`; no public service/action.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| LOC-001 | T0 `STATIC` | Confirm four GPIO channel pairs, signs/CPR/decoding, wheel geometry, IMU bus/address/mode, EKF masks/timeouts/covariances, wheel `publish_tf=false`, EKF sole odom TF, and `use_vo=false` production default. |
| LOC-002 | T1 `UNIT`/`PC` | Build/tests pass for encoder/IMU models, mecanum/odom math, covariance, config, health, topic/frame/TF and EKF assets. |
| LOC-003 | T2/T3 `HARDWARE-NON-ACTUATING` | Stationary launch: all four encoders and IMU initialize, timestamps/rates/frames/covariance are finite, `/odometry/filtered` and one odom TF are current; stationary drift is measured and recorded. |
| LOC-004 | T6 `FAULT-INJECTION` | With no motion, disconnect/stale/invalid encoder or IMU using safe supported methods; health/readiness degrades, covariance/state remains honest, duplicate TF is rejected, and no stale input is treated current. |
| LOC-005 | T4 `HARDWARE-ACTUATING` | Wheels raised: correlate each encoder direction with each wheel for forward/reverse and mecanum patterns; wrong channel/sign blocks floor tests. |
| LOC-006 | T4/T5 `HARDWARE-ACTUATING` | Guarded measured course: record straight distance scale/drift, strafe scale/drift, yaw rotation/heading, return-to-zero and stationary drift. Acceptance thresholds must come from approved repository criteria; otherwise measurement is recorded and acceptance remains BLOCKED. |
| LOC-007 | T5 `INTEGRATION` | Validate control/mapping/nav consume fresh filtered odometry; SLAM/AMCL alone owns `map -> odom`; sensor dropout closes readiness and motion is stopped/cancelled through owning systems. |
| LOC-008 | T7 `RECOVERY` | Restart sensors/EKF/localization; origin/reset behavior matches contract, TF never duplicates/jumps unexpectedly, stale history is not replayed. After VO plan passes, repeat selected tests with explicit VO profile and compare health/covariance before enabling fusion. |

## Pass criteria

All channels/signs/scales match reviewed geometry; IMU orientation is correct; covariance/freshness is honest; exactly one odom TF owner; measured drift/scale meets approved criteria; dropout/restart is controlled; VO stays gated until passed.

## Blocked criteria

Geometry/sign/mount not locked, missing physical course/threshold, hardware/build unavailable, duplicate transform unresolved, or VO not accepted for fusion.

## Failure criteria

Wrong sign/scale/frame, non-finite or implausible covariance, stale/duplicate/jumping TF, dropout shown healthy, reset replay, or unapproved VO fusion.

## Abort criteria

STOP on localization jump/loss, TF discontinuity, wrong direction, uncontrolled drift, safety stale, collision risk, or inability to stop.

## Evidence to retain

Commit/config/geometry, GPIO/channel/sign table, rates/timestamps/covariance, TF graph, bags/CSV/plots, course references and measurements, drift/scale results, fault/restart/VO comparison, operators/reviewer.

## Regression triggers

Encoder hardware/GPIO/sign/CPR/scale; wheel radius/base/track; IMU/mount/mode/calibration; covariance/EKF mask/timeout/rate; frame/TF; VO estimator/config/fusion enable; geometry.

## Current validation status

Automated source tests and historical baseline exist. Current hardware signs, geometry/scale, IMU orientation, EKF/TF, drift/dropout/restart and VO fusion are not acceptance-validated.

## Related documentation

- [Localization package](../packages/savo_localization.md)
- [Localization architecture](../architecture/localization_architecture.md)
- [Base plan](base_test_plan.md)
- [VO plan](vo_test_plan.md)
