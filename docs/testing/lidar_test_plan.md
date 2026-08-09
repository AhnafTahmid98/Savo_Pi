# LiDAR Test Plan

## Objective

Verify exclusive RPLIDAR A1 ownership, valid `LaserScan` production, health/freshness, filtering, restart, and mapping/Nav2 compatibility.

## Scope

`savo_lidar` driver, filter, sectors, watchdog, health, serial lifecycle, `laser_frame`, and consumers. No fabricated rate is used: configured expectations are checked, and the physical rate is measured.

## Test ownership

Software tests: package maintainer. Hardware/integration: Core integrator with test operator.

## Safety classification

LID-001–003 are `STATIC`/`UNIT`/`PC`; LID-004–007 are `HARDWARE-NON-ACTUATING`, `INTEGRATION`, `FAULT-INJECTION`, or `RECOVERY`, all `NO-MOTION`.

## Preconditions

Current commit/build, unique serial owner, locked mount/frame for acceptance, control `STOP`, scanner mechanically clear, and correct serial permissions.

## Required hardware

Core Pi, installed RPLIDAR A1, approved power/USB-serial path, stationary fixtures at measured ranges, and optional disconnect method that does not damage the port.

## Required software / configuration

`lidar_driver.yaml`, selected profile, topics/diagnostics config, description TF, mapping/Nav2 profiles.

## Interfaces under test

`/scan`, `/scan_filtered`, sector scans, `/savo_lidar/{state,health,watchdog_state,state_summary,scan_quality,heartbeat}`, and `laser_frame`. No service/action is owned.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| LID-001 | T0 `STATIC` | Confirm `/dev/ttyUSB0`/115200 configuration, one driver owner, `laser_frame`, configured range `0.15–12.0 m`, expected-rate health policy, timeout/reconnect bounds, and mapping/Nav2 topic consistency. |
| LID-002 | T1 `UNIT`/`PC` | Build/test package; dry-run, angle/range/rate filters, invalid samples, QoS, frames, sectors, watchdog, diagnostics, and stale policy tests pass. |
| LID-003 | T2 `TARGET-NON-HARDWARE` | Launch dry-run/health test profiles with no device; absence is unhealthy and never publishes falsely healthy live scan. |
| LID-004 | T3 `HARDWARE-NON-ACTUATING` | Detect the intended device; initialize/stop cleanly; record identity, actual scan rate/angular coverage/range validity, timestamps, frame, invalid sample behavior, and CPU. |
| LID-005 | T4 `HARDWARE-NON-ACTUATING` | Compare measured fixtures with raw/filtered scans; filtering must retain safety/navigation obstacles and reject only configured invalid/out-of-scope samples. |
| LID-006 | T5 `INTEGRATION` | With the robot stationary, mapping and LiDAR-only Nav2 receive fresh compatible scans and TF; no duplicate driver or frame mismatch exists. |
| LID-007 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Safely stop/restart and unplug/replug; stale health/readiness closes within configured policy, scanner motor stops on shutdown, reconnect is bounded, and old samples are not reused. |

## Pass criteria

One device owner; finite current scans with correct frame/timing; filters match active config; loss becomes stale/unready; shutdown stops acquisition; mapping/Nav2 compatibility is demonstrated.

## Blocked criteria

Device/permissions/locked mount/target dependency unavailable or an approved physical scan-rate baseline has not been established for acceptance.

## Failure criteria

Wrong device/frame, stale data shown current, required obstacle removed, duplicate owner, uncontrolled reconnect loop, or scanner fails to stop.

## Abort criteria

Stop and de-energize on mechanical obstruction, unusual heat/noise, unstable cabling, or electrical fault. No robot motion is permitted by this plan.

## Evidence to retain

Device identity, config, topic/rate/range/angle samples, fixture measurements, TF snapshot, diagnostics, mapping/Nav2 input evidence, disconnect/recovery timeline, logs, and result form.

## Regression triggers

Device/serial identity, mount/frame, scan mode/rate, filtering/range/angle, timeout/reconnect, QoS, mapping or Nav2 costmap source.

## Current validation status

Source tests and historical hardware baseline exist; current live scan, mount, reconnect, and integration regression are `NOT RUN`/hardware-dependent.

## Related documentation

- [LiDAR package](../packages/savo_lidar.md)
- [Perception architecture](../architecture/perception_architecture.md)
- [Navigation plan](navigation_test_plan.md)

