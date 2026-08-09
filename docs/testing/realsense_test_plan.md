# RealSense Test Plan

## Objective

Verify Edge ownership, identity, RGB-D stream profiles, frames/timestamps, health, front-depth extraction, device-loss recovery, and dependencies without conflating stream validity with obstacle-cloud/Nav2 acceptance.

## Scope

External `realsense2_camera`, package monitors/health, aligned color/depth/info/points, and `/depth/min_front_m`. VO and filtered obstacle cloud are verified in their own plans.

## Test ownership

Edge software maintainer owns RLS-001–003; camera/integration operator owns RLS-004–007.

## Safety classification

RLS-001–003 are `STATIC`/`UNIT`/`PC`; RLS-004–007 are `HARDWARE-NON-ACTUATING`, `INTEGRATION`, `FAULT-INJECTION`, or `RECOVERY`, all `NO-MOTION`.

## Preconditions

Edge STOP/safe-idle, approved USB path, configured D435 serial checked against the installed unit, current mount geometry, adequate bandwidth, and privacy handling.

## Required hardware

Edge Pi, D435, approved USB3 cable/port, static depth fixtures, and safe disconnect/reconnect access.

## Required software / configuration

Librealsense/`realsense2_camera`; D435, minimal, VO, point-cloud and health profiles. Current repository serial is `801212070967`; it is a configuration assertion to verify, not proof of the attached device.

## Interfaces under test

Configured `/camera/camera/...` color/depth/aligned images, CameraInfo and point cloud; `/savo_realsense/status`, diagnostics, and `/depth/min_front_m`. Fixed TF remains owned by `savo_description` because RealSense `publish_tf=false`.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| RLS-001 | T0 `STATIC` | Verify serial binding, `848x480x30` depth and `640x480x30` color configuration, alignment/sync, profile-specific point cloud, `publish_tf=false`, health timeout, ROI/range/percentile, and one Edge device owner. |
| RLS-002 | T1 `UNIT`/`PC` | Package build/tests pass for topics/frames, serial, stream status/timing, depth math, diagnostics, profiles, and C++ defaults. |
| RLS-003 | T2 `TARGET-NON-HARDWARE` | Launch argument/config parsing passes without claiming a camera; missing streams become unhealthy and front depth invalid/stale. |
| RLS-004 | T3 `HARDWARE-NON-ACTUATING` | Enumerate D435; record serial, firmware report, USB mode, port/cable, temperatures if available, and selected profile. Identity mismatch blocks production start. |
| RLS-005 | T4 `HARDWARE-NON-ACTUATING` | Measure color/depth rates/resolutions, timestamps, alignment, CameraInfo, frames, depth fixture values, health, front-depth ROI and CPU/bandwidth over a representative stationary interval. Use configured rates as criteria; do not invent tolerance. |
| RLS-006 | T5 `INTEGRATION` | Feed the validated streams separately to VO and optional cloud producer. Prove VO inputs are synchronized; do not mark D435 self-filter/voxel acceptance from this result. |
| RLS-007 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Safely unplug/replug or stop driver; status becomes stale, dependent VO/cloud readiness closes, no stale depth remains valid, restart reacquires the bound unit, and repeated resets/bandwidth instability are recorded. |

## Pass criteria

Correct bound device over approved USB mode; configured streams/frames/timestamps/alignment/health are stable; front depth is plausible; loss is honest; dependencies close and recover safely.

## Blocked criteria

Serial mismatch, absent hardware/USB3/dependencies, unmeasured mount, privacy restriction, or temperature/bandwidth criteria not yet approved for acceptance.

## Failure criteria

Wrong device/profile/frame, timestamp regression, false health, depth outside fixture/config policy, duplicate owner, stale dependent output, or unstable recovery.

## Abort criteria

Stop on overheating, electrical/USB damage, repeated uncontrolled reset, privacy breach, or any unexpected actuation.

## Evidence to retain

Enumeration and firmware output, USB topology, serial/profile/config, topic metadata/rates, frame/TF snapshot, depth measurements, health/temperature/CPU/bandwidth, device-loss timeline, and logs.

## Regression triggers

D435/cable/port/serial/firmware, mount/TF, stream/rate/resolution/alignment/sync/QoS, ROI/depth policy, reset behavior, VO/cloud dependency.

## Current validation status

Source/retained PC evidence exists. Current USB3, identity, streams, thermal/bandwidth, front depth, and restart are hardware-dependent and `NOT RUN` in Phase 7.

## Related documentation

- [RealSense package](../packages/savo_realsense.md)
- [RealSense setup](../setup/realsense_setup.md)
- [VO plan](vo_test_plan.md)
- [Perception plan](perception_test_plan.md)

