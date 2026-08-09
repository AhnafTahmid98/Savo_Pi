# Head Test Plan

## Objective

Verify safe pan/tilt ownership and limits, dynamic head TF, Pi Camera streaming, AprilTag evidence, scan integration, and restart behavior.

## Scope

`savo_head` controller, PCA9685 channels 15/14, scan/status/TF, `gscam`/libcamera path, detector, and `ConfirmAprilTag`. Location approval is out of scope.

## Test ownership

Software maintainer owns HED-001–003. Hardware stages require a head operator and a separate safety observer when the base is powered.

## Safety classification

HED-001–003 are `STATIC`/`UNIT`/`PC`; HED-004 is `HARDWARE-NON-ACTUATING`; HED-005–007 are `HARDWARE-ACTUATING`/`WHEELS-RAISED`, `INTEGRATION`, and `RECOVERY`. Base control remains STOP.

## Preconditions

Measured mount/centers/limits, cable-clearance inspection, base STOP, lowest safe servo range, current calibration, camera privacy approval, and one PCA9685 owner strategy reviewed with `savo_base`.

## Required hardware

Core Pi, PCA9685/servos, head mechanism, Pi Camera 2 NoIR, tag fixture, physical clearance, and power isolation.

## Required software / configuration

Head servo/scan/frame/camera/tag YAML, description geometry, gscam/libcamera dependencies, `savo_msgs` action interface.

## Interfaces under test

Pan/tilt command/state, scan command/state and Trigger services, camera image/info, head diagnostics, AprilTag observations, `/savo_head/apriltag/confirm`, and dynamic head TF chain.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| HED-001 | T0 `STATIC` | Confirm channels 15/14, configured limits/centers/watchdog, dynamic TF ownership exclusion from description, camera backend/profile, and typed tag action. |
| HED-002 | T1 `UNIT`/`PC` | Build/tests pass for limits, calibration, scan patterns, TF calibration gate, camera health/launch, and tag contracts/runtime. |
| HED-003 | T2 `TARGET-NON-HARDWARE` | With servo output disabled, invalid/out-of-range commands clamp/reject; stale command policy, scan state, camera failure, and unhealthy calibration suppress valid TF/tag confirmation. |
| HED-004 | T3 `HARDWARE-NON-ACTUATING` | Detect PCA/camera without moving servos; record devices, camera `640x480@30` configured profile, image frames/timestamps/health, and no duplicate PCA owner process. |
| HED-005 | T4 `HARDWARE-ACTUATING` | Starting at minimum safe range, verify pan/tilt center then conservative endpoints and soft limits; observe stall/current/cable clearance. Wrong direction or limit immediately aborts. |
| HED-006 | T5 `INTEGRATION` | Run a reduced scan; verify joint state and dynamic TF track physical head; confirm image optical orientation and stable AprilTag evidence; exercise `ConfirmAprilTag` rejection for wrong/stale/unstable/moving evidence and Scan360 binding if selected. |
| HED-007 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Disconnect camera or disable servo backend safely, cancel scan, and restart; state becomes unhealthy, no stale confirmation is accepted, servos center/stop per policy, and restart does not jump outside limits. |

## Pass criteria

Physical center/direction/soft limits match reviewed calibration; cable clearance is safe; one dynamic TF authority; camera and tag evidence are fresh/correct; failures reject confirmation and restart safely.

## Blocked criteria

Unmeasured limits/mounts, unresolved shared-PCA initialization, absent camera/servo dependency, or no controlled actuation authorization.

## Failure criteria

Wrong motion, unclamped command, stall, cable tension, duplicate TF/PCA ownership, wrong optical frame, stale tag acceptance, or unsafe restart.

## Abort criteria

Immediately remove servo power on obstruction, cable tension, stall/buzzing, excessive heat/current, wrong limits, collision, or unexpected base motion.

## Evidence to retain

Calibration/geometry digest, channel/owner record, device identity, camera samples, limit measurements, low-range video, TF/joint trace, tag/action logs, restart results, operators/reviewer.

## Regression triggers

PCA channel/init policy, limits/centers, mount/TF, scan pattern, camera backend/profile/mount, tag size/thresholds/action contract, shutdown policy.

## Current validation status

Source tests and historical hardware evidence exist. Current limits, shared PCA behavior, camera, TF, scan, and AprilTag integration require hardware regression.

## Related documentation

- [Head package](../packages/savo_head.md)
- [Sensor mounting](../hardware/sensor_mounting.md)
- [Locations plan](locations_test_plan.md)

