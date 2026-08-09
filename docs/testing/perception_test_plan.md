# Perception Test Plan

## Objective

Verify the Core near-field safety path independently from the optional Edge D435 obstacle-cloud path. Core must fail closed from `/cmd_vel` to `/cmd_vel_safe`; D435 cloud never replaces near-field safety.

## Scope

Core ToF mux/left-right ownership, ultrasonic, range health, thresholds/staleness, STOP/slowdown fusion and command gate; Edge cloud frame, finite/range/floor/self filters, freshness/output and guarded Nav2 integration.

## Test ownership

Perception maintainer owns PER-001–006. Hardware safety tests require motion-test lead and E-stop operator. Edge cloud/Nav2 tests also require navigation reviewer.

## Safety classification

PER-001–004 and PER-007–008 are `STATIC`, `UNIT`, `PC`, `TARGET-NON-HARDWARE`, or `HARDWARE-NON-ACTUATING`; PER-005–006 are `HARDWARE-ACTUATING` / `WHEELS-RAISED` then `GUARDED-FLOOR-MOTION`; PER-009–010 are `INTEGRATION`, `FAULT-INJECTION`, or `RECOVERY`.

## Preconditions

Core control STOP, measured sensor mounting/fixtures, locked geometry for motion/self-filter acceptance, known active thresholds recorded without alteration, base/control passed before actuation, D435 voxel disabled by default.

## Required hardware

Core TCA9548A with left/right VL53L1X, ultrasonic sensor, measured obstacle fixtures, stands/E-stop; Edge D435 for optional cloud tests.

## Required software / configuration

Core sensor/safety/gate/health profiles and topics; Edge cloud-filter profile; description TF; RealSense and LiDAR-only navigation baseline.

## Interfaces under test

Core range, safety, health and `/cmd_vel_safe` topics from [the package page](../packages/savo_perception.md); Edge obstacle cloud/status. No service/action/TF authority.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| PER-001 | T0 `STATIC` | Verify Core sensor GPIO/I2C/mux channels and left/right identity, threshold/stale policy, command timeout, `/cmd_vel -> /cmd_vel_safe`, clamp-only behavior and C++ production nodes. |
| PER-002 | T1 `UNIT`/`PC` | Build/tests pass for range models/acquisition contracts, safety fusion/gating/direction logic, stale/invalid state, command timeout, topics, and synthetic point-cloud filters. |
| PER-003 | T2 `TARGET-NON-HARDWARE` | With synthetic inputs and no motors, test forward/reverse/left/right/strafe/yaw gating against appropriate sensor sectors, slowdown clamp, STOP, obstacle removal, zero command, invalid/NaN ranges, stale ranges and stale command. |
| PER-004 | T3 `HARDWARE-NON-ACTUATING` | Measure left/right ToF and ultrasonic identity/range/timestamps against fixtures; disconnect each sensor; required invalid/stale data must produce conservative state. Record false readings and active threshold values. |
| PER-005 | T4 `HARDWARE-ACTUATING` | Wheels raised: trace bounded `/cmd_vel` through safety state and `/cmd_vel_safe`; introduce/remove controlled fixtures for each direction. STOP/slowdown and timeout match active policy; removal does not replay a stale command. |
| PER-006 | T5 `INTEGRATION` | Guarded floor only after prerequisite PASS: measure actual slowdown/stop behavior and false-positive/negative envelope. If formal stop-distance criteria are absent, record measurement and leave acceptance BLOCKED. |
| PER-007 | T0/T2 `STATIC`/`PC` | Verify Edge cloud is optional/off by default; frame and finite/range/height/floor/self crops, stale timeout, output/health and D435 validation gate match config. |
| PER-008 | T3 `HARDWARE-NON-ACTUATING` | With stationary D435, measure raw/filtered cloud, frame/timestamps/rate and real robot self/floor removal across mount/view; retain nearby external obstacles. |
| PER-009 | T5 `INTEGRATION` | Only after PER-008 review, feed cloud to the guarded Nav2 voxel profile while LiDAR remains validated clearing source; confirm costmap marking, no unsafe clearing/self/floor artifacts, and readiness closure on stale cloud. |
| PER-010 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Disconnect/restart Core sensor(s), gate and D435; Core fails closed, optional voxel disables/falls back to validated LiDAR-only profile, and recovery uses fresh data. |

## Pass criteria

Core sensor identities/ranges are plausible; required stale/invalid input and command timeout fail closed; direction-specific gating never increases commands; obstacle removal is controlled. Edge cloud passes independent physical self/floor/freshness/Nav2 criteria before its gate is enabled.

## Blocked criteria

Unmeasured mounts/thresholds/stop criterion, unavailable safe fixtures/operators, unresolved sensor identity, D435 mount/self-filter not measured, or LiDAR-only navigation not yet accepted.

## Failure criteria

Wrong sensor side, unsafe direction permitted, stale/invalid input shown safe, `/cmd_vel_safe` bypass/increase, command persists, self/floor points enter costmap, or D435 loss leaves voxel readiness true.

## Abort criteria

E-stop/remove power for failure to stop, unexpected motion, fixture/person hazard, stale safety state, sensor overheating/electrical fault, or unsafe costmap behavior.

## Evidence to retain

Commit/config/geometry, mux/GPIO identity, fixture measurements and timestamps, command/safety/safe traces, disconnect/recovery timeline, wheel-raised/floor video, stop measurements, raw/filtered cloud bags/images, costmap evidence, reviewers.

## Regression triggers

Sensor/mux/GPIO/mount replacement or move; thresholds/stale/timeout/direction gate; `/cmd_vel` contracts; D435/mount/profile/filter bounds/frame; Nav2 voxel/costmap/clearing policy; geometry.

## Current validation status

Core source tests and historical physical baseline exist; current thresholds/identity/stale/stop behavior require hardware regression. Edge cloud is source/synthetic-validated only and production voxel remains BLOCKED.

## Related documentation

- [Perception architecture](../architecture/perception_architecture.md)
- [Safety architecture](../architecture/safety_architecture.md)
- [RealSense plan](realsense_test_plan.md)
- [Navigation plan](navigation_test_plan.md)
