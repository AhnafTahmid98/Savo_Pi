# Bringup Test Plan

## Objective

Verify that `savo_bringup` composes the approved Core and Edge graphs, starts fail-closed, and reports readiness without granting motion authority.

## Scope

Role, mode, profile, feature-flag, readiness, heartbeat, geometry, D435, duplicate-owner, startup, shutdown, partial-failure, and restart contracts. Component behavior remains in its owning plan.

## Test ownership

Software lead owns BRG-001–006; Core/Edge integrator owns BRG-007–010. A safety operator is required only if later component plans permit actuation.

## Safety classification

BRG-001–006 are `STATIC`, `SOURCE-CONTRACT`, or `PC` / `NO-MOTION`. BRG-007–010 are `TARGET-NON-HARDWARE`, `INTEGRATION`, or `RECOVERY` / `NO-MOTION`. This plan never authorizes movement.

## Preconditions

- Exact commit, role environment, ROS domain, mode, profile, and feature flags are recorded.
- Core control remains `STOP`; drivetrain power is isolated for live graph tests.
- No generic and role-specific service may own the same graph concurrently.

## Required hardware

None for BRG-001–006. Core and Edge target computers and their dedicated network are required for BRG-007–010; peripherals may remain disconnected if absence is expected and honestly reported.

## Required software / configuration

ROS 2 Jazzy; role arrays in `deploy/{core,edge}/env_*.sh`; `savo_bringup` launch/config; locked geometry only for motion profiles; D435 voxel validation flag remains false until separately approved.

## Interfaces under test

`/savo_bringup/{core,edge}/{state,ready,heartbeat}`, diagnostics, component freshness inputs, launch arguments, and `/var/lib/robot_savo` path forwarding. Bringup owns no command/service/action authority.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| BRG-001 | T0 `STATIC` | Run `bash deploy/common/validate_full_bringup.sh`; required assets, parsers, role arrays, paths, and retired-package exclusion pass. |
| BRG-002 | T0 `SOURCE-CONTRACT` | Inspect `robot_bringup.launch.py`: only `core`/`edge` roles; invalid role/mode/profile is rejected; supported modes are `safe_idle`, `manual`, `manual_mapping`, `autonomous_mapping`, `saved_map_navigation`, `diagnostics`; supported profiles are `bench`, `lidar_only`, `lidar_d435_voxel`, `production`. |
| BRG-003 | T0 `SOURCE-CONTRACT` | Confirm safe-idle, Core `STOP`, geometry-required, and `d435_voxel_validated=false` defaults; provisional geometry and unvalidated voxel requests fail closed. |
| BRG-004 | T1 `UNIT`/`PC` | Build/test `savo_bringup`; package launch/contract/runtime tests pass with `colcon test-result --verbose`. |
| BRG-005 | T2 `TARGET-NON-HARDWARE` | Generate `--show-args` for production and mode launches; required package includes match role arrays and optional speech/UI/cloud flags do not create duplicate owners. |
| BRG-006 | T2 `SOURCE-CONTRACT` | Verify readiness requires configured fresh component states, heartbeat advances, stale/malformed inputs close ready, and Edge readiness cannot grant Core motion. |
| BRG-007 | T3 `INTEGRATION` | Core-only safe-idle: one graph, `STOP`, zero `/cmd_vel_safe`, honest readiness, clean shutdown. |
| BRG-008 | T3 `INTEGRATION` | Edge-only safe-idle: one RealSense/VO/bridge owner; optional features match flags; honest readiness and clean shutdown. |
| BRG-009 | T5 `INTEGRATION` | Start Core then Edge; confirm discovery, independent role readiness/freshness, distributed startup order, no duplicate nodes/TF owners, and safe shutdown in both orders. |
| BRG-010 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | In a non-actuating setup stop/restart one required component, sever/recover DDS, and restart each role; readiness must drop stale, commands stay zero, and recovery requires fresh evidence rather than replay. |

## Pass criteria

Every in-scope test is PASS with evidence; startup stays `STOP`; invalid/provisional/unvalidated selections fail closed; each required owner starts once; readiness is fresh and honest.

## Blocked criteria

ROS/target/network prerequisite missing, target builds unavailable, or geometry/hardware prerequisite prevents the intended profile from validly starting.

## Failure criteria

Unsupported input is accepted, readiness stays true on stale evidence, duplicate ownership occurs, a nonzero command appears, or shutdown/restart leaves an unsafe owner.

## Abort criteria

Follow [failure and abort criteria](failure_and_abort_criteria.md). Immediately isolate drivetrain power on any unexpected actuation or inability to confirm STOP.

## Evidence to retain

BRG test IDs, commit, role arrays, exact launch arguments, environment, graph/topic/diagnostic snapshots, readiness/heartbeat timeline, validator and `colcon` logs, shutdown/restart logs.

## Regression triggers

Role arrays, launch includes/defaults, readiness sources/timeouts, modes/profiles, geometry/D435 gates, runtime paths, service ownership, or component feature flags. See [regression matrix](regression_matrix.md).

## Current validation status

BRG-001 is source-validated by the Phase 7 run. Package/target/integration results require their execution records; BRG-007–010 are `NOT RUN` for current-source hardware targets unless referenced evidence says otherwise.

## Related documentation

- [Bringup package](../packages/savo_bringup.md)
- [Bringup readiness architecture](../architecture/bringup_readiness_state_machine.md)
- [Full robot plan](full_robot_test_plan.md)

