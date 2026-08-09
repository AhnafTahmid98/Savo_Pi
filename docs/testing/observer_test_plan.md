# Observer Test Plan

## Objective

Verify the workstation observer builds, discovers Core/Edge, presents fresh/stale telemetry, maps and TF, reconnects safely, and remains strictly read-only.

## Scope

Source validator, telemetry/dashboard/browser, constrained RViz profiles, discovery, freshness/offline state, performance, and reconnect. No robot mutation interface is allowed.

## Test ownership

Observer maintainer owns OBS-001–004; workstation/network integrator owns OBS-005–007.

## Safety classification

All tests are `STATIC`, `UNIT`, `PC`, `TARGET-NON-HARDWARE`, `INTEGRATION`, or `RECOVERY` / `NO-MOTION`.

## Preconditions

Trusted ROS 2 Jazzy workstation, matching domain/RMW/network, robot STOP for connection tests, approved browser/bind policy, and no command tooling loaded.

## Required hardware

Workstation/display/network; no robot actuation hardware is required.

## Required software / configuration

Observer package/config/profiles/RViz/dashboard assets and deployment scripts. It is absent from both Pi production role arrays.

## Interfaces under test

Configured telemetry subscriptions, TF/map visualization and observer-local snapshot/status. No robot command publisher, service client, action client, SetGoal, SetInitialPose, PublishPoint, or teleop tool.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| OBS-001 | T0 `STATIC` | Run `bash deploy/observer/validate_observer.sh`; required assets/parsers pass and mutation source/RViz tools are absent. |
| OBS-002 | T1 `UNIT`/`PC` | Build/test `savo_observer`; freshness/snapshot, launch/config, dashboard/RViz and read-only contracts pass. |
| OBS-003 | T2 `TARGET-NON-HARDWARE` | Generate launch args for observer/RViz/dashboard/full profiles; browser binds only as configured and missing telemetry displays offline/stale. |
| OBS-004 | T0 `SOURCE-CONTRACT` | ROS/source graph inventory proves no motion/goal/initial-pose/map/location/service/action authority and observer is not in Core/Edge role arrays. |
| OBS-005 | T5 `INTEGRATION` | Discover both Pis; render expected Core/Edge telemetry, map and connected TF with correct freshness; RViz tools remain constrained. |
| OBS-006 | T5 `INTEGRATION` | Record bandwidth, CPU/memory, dashboard/RViz/browser responsiveness for standard/low-bandwidth/mobile profiles; acceptance threshold must be approved if not defined. |
| OBS-007 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Disconnect/reconnect network and stop/restart producers/observer; stale/offline appears, old data is not shown current, reconnect restores only fresh data, robot authority is unchanged. |

## Pass criteria

Validator/build/tests pass; correct telemetry/map/TF is visible and stale-labeled; reconnect is bounded; performance is recorded; no mutation path or unsafe RViz tool exists.

## Blocked criteria

Workstation ROS/dependencies/browser/network unavailable, domain/RMW not approved, or performance acceptance threshold missing.

## Failure criteria

Any command/client/action/unsafe RViz tool, stale data shown current, incorrect TF/map, uncontrolled external bind, or observer affects robot operation.

## Abort criteria

Stop observer and disconnect it on unexpected publication/client, excessive network/CPU impact, credential exposure, or false safety/readiness presentation.

## Evidence to retain

Validator/build/test logs, commit/workstation/ROS/network, graph report, screenshots with revision/state, freshness/reconnect timeline, profile/performance measurements, browser/RViz configuration.

## Regression triggers

Any publisher/client/action/tool, telemetry/topic/type/freshness set, RViz/dashboard/browser assets, bind/security, network/domain/RMW, package role membership.

## Current validation status

OBS-001 is source-validated in Phase 7. Actual workstation build/network/dashboard/RViz integration and performance remain target-dependent.

## Related documentation

- [Observer package](../packages/savo_observer.md)
- [Network architecture](../architecture/network_architecture.md)
- [UI plan](ui_test_plan.md)

