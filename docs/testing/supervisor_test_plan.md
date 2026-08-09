# Supervisor Test Plan

## Objective

Verify fail-closed robot-wide readiness, arming, mission/location authorization, map context, fault latch, authority revocation, persistence, and passive shutdown request while preserving the distinction between permission and execution.

## Scope

`supervisor_node`, component freshness, system/mission state, typed services, cancellation clients, state file, restart/corruption, and no direct motor execution.

## Test ownership

Supervisor maintainer owns SUP-001–006; system integration/safety reviewer owns SUP-007–010.

## Safety classification

SUP-001–009 are `STATIC`, `UNIT`, `PC`, `TARGET-NON-HARDWARE`, `INTEGRATION`, `FAULT-INJECTION`, or `PERSISTENT-STATE` / `NO-MOTION`; SUP-010 mission revoke is physical only under separately passed motion plans.

## Preconditions

Isolated copied state for destructive/corruption cases, exact policy/config, STOP, representative Core/Edge readiness fixtures, and operation owners configured to enforce supervisor evidence.

## Required hardware

None through source/runtime simulation. Core/Edge hosts for distributed integration; physical system only for separately authorized mission-cancel validation.

## Required software / configuration

`supervisor.yaml`, location authorization policy, `/var/lib/robot_savo/supervisor/system_state.json` production path, `savo_msgs`, and action dependencies.

## Interfaces under test

Supervisor state/capability/mission authority/heartbeat/fault/shutdown summaries; authorize operation/location, update map context, manage system state services; cancellation clients. No motor publisher/action execution owner.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| SUP-001 | T0 `STATIC` | Verify one Core owner, startup unarmed, required/optional sources and timeouts, persistent path, map/fault policy, typed services, cancellation set, and no motor/shell execution. |
| SUP-002 | T1 `UNIT`/`PC` | Build/tests pass for parsing, freshness/readiness, transitions, authority, map/location policy, state store, edge supervision, services and runtime contracts. |
| SUP-003 | T2 `TARGET-NON-HARDWARE` | Start with an isolated state file; initial state is non-armed, capabilities match inputs, and missing/stale/malformed evidence denies authority. |
| SUP-004 | T2 `TARGET-NON-HARDWARE` | Exercise status, arm/disarm, authorize, and map-context services with valid/invalid transitions; denial reason is explicit; disarm does not execute operation-owned behavior. |
| SUP-005 | T6 `FAULT-INJECTION` | Inject Core/Edge staleness, safety fault, map/release mismatch and active-mission evidence; latch fault, revoke authority, request typed cancellation, and prevent automatic resumption. |
| SUP-006 | T6 `FAULT-INJECTION` | Issue low-power/controlled shutdown condition in simulation; supervisor revokes/persists/publishes request but never shells out or directly powers off. |
| SUP-007 | T7 `PERSISTENT-STATE`/`RECOVERY` | Restart with valid copied state; generation/context/latch persist as specified and unsafe commands are not replayed. |
| SUP-008 | T7 `FAULT-INJECTION` | Corrupt/truncate/replace copied state only; startup fails closed or reports recovery-required. Never damage production state. |
| SUP-009 | T5 `INTEGRATION` | On Core/Edge safe-idle, verify actual freshness/capabilities, arm/disarm/latch/clear, map context and optional Edge degradation; executors and bridge respect denials. |
| SUP-010 | T5 `INTEGRATION` | Under separately authorized mission test, revoke authority during mapping/nav/rotation/coverage/tag confirmation; each owner cancels/stops through its own interface and control/perception/base remain decisive. |

## Pass criteria

Unarmed startup; explicit prerequisites for authority; stale/fault/mismatch revokes and latches; clear is explicit; persistence is atomic/fail-closed; supervisor only grants permission and requests cancellation/shutdown.

## Blocked criteria

No isolated state environment, incomplete executor cancellation contract, unavailable distributed targets, or physical mission prerequisites not passed.

## Failure criteria

Unsafe auto-arm/replay, authority with stale/mismatched data, unlatching fault, corrupt state accepted, direct motor/shell execution, or executor ignores current authority.

## Abort criteria

STOP and isolate operations on unexpected authorization, failure to revoke/cancel, state corruption against production data, continued motion, or shutdown side effect outside the test plan.

## Evidence to retain

Policy/config/commit, isolated state before/after/digest/generation, service cases/reasons, readiness/fault timeline, cancellation correlation, executor observations, restart/corruption logs, review.

## Regression triggers

Required sources/timeouts, arming/authorization/fault policy, map context, state schema/path, cancellation list, shutdown semantics, `savo_msgs`, or operation-owner enforcement.

## Current validation status

Implemented/source-tested. Current two-Pi arming/latch/revoke/cancel, persistence and shutdown integration are `NOT RUN` for this Phase 7 documentation execution.

## Related documentation

- [Supervisor package](../packages/savo_supervisor.md)
- [Safety architecture](../architecture/safety_architecture.md)
- [Motion authority](../architecture/motion_authority_model.md)

