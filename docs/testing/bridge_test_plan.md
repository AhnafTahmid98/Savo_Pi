# Bridge Test Plan

## Objective

Verify that `savo_bridge` is a credential-checked, bounded, typed SavoMind boundary and cannot become a generic ROS, shell, supervisor, operator-approval, or motor bypass.

## Scope

Unix socket parent/permissions/peer credentials, snapshot/freshness, protocol limits/correlation, STOP, cancellation, bounded teleop, named navigation, mapping controls/queries, unavailable dependencies, restart, and authority-negative tests.

## Test ownership

Bridge/security maintainer owns BRD-001–006; Edge/Core integration owner owns BRD-007–009.

## Safety classification

BRD-001–008 are `STATIC`, `UNIT`, `SOURCE-CONTRACT`, `PC`, `TARGET-NON-HARDWARE`, `INTEGRATION`, or `FAULT-INJECTION` / `NO-MOTION`. BRD-009 bounded live operations require their owning physical plans; this plan alone does not authorize actuation.

## Preconditions

Exact allowed UID/GID and socket model reviewed; isolated test socket/data; Core control STOP; no real motion action accepted during protocol tests; active map context only when explicitly testing navigation.

## Required hardware

None through BRD-006. Edge/Core hosts and dedicated network for integration; physical robot only under separately authorized control/navigation/mapping plans.

## Required software / configuration

`savo_bridge.edge.yaml`, `/run/savo_bridge` provisioning/service model, test peer identities, typed ROS dependencies, and SavoMind protocol client.

## Interfaces under test

`/run/savo_bridge/command.sock`, `snapshot.json`, explicitly modeled control publications, locations/mapping services, guarded navigation/mapping actions, and observed telemetry. Never `/cmd_vel_safe`, arbitrary graph forwarding, or shell.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| BRD-001 | T0 `STATIC` | Verify parent directory provisioning, socket `0660`, configured allowed UID `10001`/deployment override, max 65536-byte request/response, timeouts/freshness, map-context and graph-evidence policy. |
| BRD-002 | T0 `SOURCE-CONTRACT` | Source/tests prove no generic topic/service/action proxy, arbitrary publication, shell execution, direct motor/safe-command path, supervisor bypass, or operator map/location approval. |
| BRD-003 | T1 `UNIT`/`PC` | Build/tests pass for framing, parse/schema/bounds, peer credentials, duplicate command IDs, graph evidence, snapshot, freshness, dispatch, STOP, teleop, navigation/cancel and mapping adapters. |
| BRD-004 | T2 `TARGET-NON-HARDWARE` | Create isolated runtime parent with intended owner/group/mode; intended peer connects; wrong UID, socket parent/symlink/permission failure, malformed/oversized/unsupported request, timeout and duplicate ID are rejected and correlated. |
| BRD-005 | T2 `TARGET-NON-HARDWARE` | Produce snapshot with commit/context/freshness; missing/stale/malformed observations are labeled and command admission closes rather than using last-known readiness. |
| BRD-006 | T6 `FAULT-INJECTION` | Simulate unavailable service/action, wrong map context, stale readiness, SavoMind disconnect, response timeout and cancellation; all fail closed with bounded reason/correlation. |
| BRD-007 | T5 `INTEGRATION` | In distributed STOP, verify typed STOP and supervisor query; inspect ROS graph to prove only documented publishers/clients/actions exist and command IDs correlate end to end. |
| BRD-008 | T7 `RECOVERY` | Restart bridge/SavoMind/network; stale socket is removed safely, permissions remain correct, duplicate replay is rejected, snapshot recovers only on fresh evidence. |
| BRD-009 | T5 `INTEGRATION` | Under separately passed control/nav/mapping prerequisites, validate bounded teleop, named-location navigation/cancel, and mapping control/query; Core readiness/authority/release gates remain decisive. |

## Pass criteria

Only approved peer and typed bounded schemas work; freshness/map/authority/dependency checks fail closed; STOP/cancel correlate; restart is safe; no generic/bypass capability exists.

## Blocked criteria

UID/GID/runtime ownership decision unresolved, target integration unavailable, or owning subsystem physical prerequisite not passed.

## Failure criteria

Unauthorized peer accepted; malformed/oversized/replayed/stale request acts; arbitrary forwarding/shell/bypass exists; wrong map admitted; timeout/cancel lacks bounded result.

## Abort criteria

Engage STOP and isolate command peer on unexpected operation, command outside bounds, authority mismatch, continued motion after cancel, or socket security failure.

## Evidence to retain

Socket stat/credentials, config, protocol cases and raw bounded metadata, graph report, snapshots, correlation IDs, service/action results, fault/restart timeline, Core authority evidence, and review.

## Regression triggers

Protocol/schema/limits, UID/GID/path/service model, ROS interface or observation set, freshness/map policy, new operation, SavoMind identity, navigation/mapping/supervisor contracts.

## Current validation status

Implemented/source-tested with prior protocol smoke evidence. Current Edge credentials/socket ownership, distributed freshness/latency/restart, and live typed operations require integration validation.

## Related documentation

- [Bridge package](../packages/savo_bridge.md)
- [SavoMind boundary](../architecture/savomind_ros_boundary.md)
- [Speech plan](speech_test_plan.md)

