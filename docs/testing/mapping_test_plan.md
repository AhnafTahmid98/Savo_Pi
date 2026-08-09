# Mapping Test Plan

## Objective

Verify manual and autonomous mapping lifecycles, guarded goal handoff/timeouts/cancellation, session artifacts and quality, semantic candidate review, and operator-approved immutable production release.

## Scope

SLAM lifecycle, mapping modes, manual save, frontier/coverage/selected-goal handoff, autonomous action/orchestration, response/execution/feedback timeouts, cancellation/recovery, Scan360 and semantic interruption where implemented, map verification/quality/review/release/active context/restart/negative release cases.

## Test ownership

Mapping maintainer owns MAP-001–007; mapping operator/safety reviewer owns MAP-008–013; release approver owns MAP-014.

## Safety classification

MAP-001–007 are `STATIC`, `UNIT`, `SOURCE-CONTRACT`, `PC`, `TARGET-NON-HARDWARE`, `FAULT-INJECTION`, or `PERSISTENT-STATE` / `NO-MOTION`. MAP-008–013 are `HARDWARE-ACTUATING`, `INTEGRATION`, `RECOVERY`, or `ACCEPTANCE` / `GUARDED-FLOOR-MOTION`; MAP-014 is `PERSISTENT-STATE`/operator approval.

## Preconditions

All static/target gates pass; locked geometry; base/control/perception/LiDAR/localization/supervisor PASS; clear mapped area and E-stop team; isolated session/release path for negative tests; manual mapping before autonomous mapping.

## Required hardware

Complete Core drivetrain/safety/localization/LiDAR system; head/camera/tags only for Scan360/semantic scope; controlled test area and reference measurements.

## Required software / configuration

SLAM Toolbox; mapping session/mode/quality/catalog/autonomous/coverage/frontier/Scan360/semantic configs; `/var/lib/robot_savo/maps`; `savo_msgs`; supervisor and locations integrations.

## Interfaces under test

Mapping status/session topics, configured mode/save/quality/catalog/review services, `RunAutonomousMapping`, coverage execution, mapped-location registration and AprilTag/rotation dependencies documented in source/package contracts. Internal interfaces are not promoted as public operator entry points.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| MAP-001 | T0 `STATIC` | Inventory current nodes/launch/config/interfaces; verify map save, quality, operator review and production release are separate gates; production paths are not `/tmp`; only implemented frontier/coverage/Scan360/semantic strategies are claimed. |
| MAP-002 | T1 `UNIT`/`PC` | Build/tests pass for mapping types/modes/session, SLAM/config, frontier/exploration, goal handoff, coverage, Scan360, semantic interruption, autonomous orchestrator/sequencer, release/quality and location review contracts/runtime. |
| MAP-003 | T2 `TARGET-NON-HARDWARE` | Launch monitor/simulated profiles; validate lifecycle transitions, status/readiness, session identity/artifact paths and clean shutdown without robot commands. |
| MAP-004 | T2 `SOURCE-CONTRACT` | Simulate selected-goal response timeout, execution timeout, stale feedback, invalid result and cancel; handoff terminates with correlated reason and no active command remains. |
| MAP-005 | T2 `PERSISTENT-STATE` | With fixture maps in isolated storage, test save completeness, manifests/hashes, verification, quality reports, missing/corrupt artifacts, immutable release construction and active identity. |
| MAP-006 | T6 `FAULT-INJECTION` | Negative release cases: provisional/wrong geometry, quality unapproved, missing operator review, map/release/hash mismatch, path traversal/corruption and stale session all reject. |
| MAP-007 | T7 `RECOVERY` | Restart session/catalog/orchestrator using isolated state; interrupted save/release remains non-active, recovery is explicit, and immutable artifacts are not rewritten. |
| MAP-008 | T3 `HARDWARE-NON-ACTUATING` | Stationary real sensors/TF/SLAM lifecycle become healthy; map updates are plausible and no duplicate `map -> odom` owner exists. |
| MAP-009 | T4/T5 `HARDWARE-ACTUATING` | Run manual mapping first through supported bringup in a guarded area; validate pause/cancel/save, safety/control/readiness and retain the session artifacts. |
| MAP-010 | T5 `INTEGRATION` | Independently verify saved map completeness, scale/alignment/coverage/artifacts and configured quality metrics. Where no approved numeric threshold exists, measurement is recorded and acceptance remains BLOCKED. |
| MAP-011 | T5 `HARDWARE-ACTUATING` | After manual workflow PASS, run bounded frontier/coverage autonomous mapping; verify authorization, handoff, feedback, timeouts, cancellation, no-go/safety behavior, completion and recovery. |
| MAP-012 | T5 `HARDWARE-ACTUATING` | If selected, run Scan360 using the typed rotate action and minimum safe head/base scope; test cancel/stale/quality and never bypass control/perception. |
| MAP-013 | T5 `INTEGRATION` | If semantic scope is selected, collect real AprilTag evidence, create a pending candidate, test interruption/confirmation/correlation and require explicit operator review through locations workflow. |
| MAP-014 | T5/T7 `ACCEPTANCE` | Authorized reviewer approves quality/release, creates immutable production release, associates exact geometry/map/location context, activates atomically and verifies restart/rollback. Automation may not approve on behalf of operator. |

## Pass criteria

Lifecycle/handoffs/timeouts/cancel are bounded; manual flow succeeds before autonomous; artifacts/hashes/quality/review are distinct and valid; unsafe/mismatched releases reject; only explicit operator approval activates immutable release.

## Blocked criteria

Any earlier motion gate not PASS, geometry unlocked, no approved quality threshold/reviewer/test area, missing hardware/target build, or semantic/Scan360 scope not authorized.

## Failure criteria

Continued motion after cancel/timeout, stale feedback accepted, corrupted/incomplete map promoted, quality/review bypass, mutable release, wrong geometry/context activated, or SavoMind/operator authority violation.

## Abort criteria

STOP/cancel on safety/localization/TF loss, unexpected/repeated recovery, collision/person hazard, map discontinuity, authority mismatch or inability to stop. Preserve session before diagnosis.

## Evidence to retain

Commit/profiles/geometry, mission/session/correlation IDs, bags/logs/TF, timeout/cancel timeline, map files/manifests/hashes/quality measurements, semantic evidence, operator review, release/active/rollback identities.

## Regression triggers

Mapping/SLAM source/config; planners/handoff/timeouts; map quality; session/artifact/storage schema; geometry/TF/LiDAR/localization; release/digest/review; locations/supervisor/action interfaces.

## Current validation status

Extensive automated coverage exists. Current target build, manual mapping, physical autonomous/coverage/Scan360/semantic workflows, approved quality thresholds and production release acceptance remain hardware/integration-dependent.

## Related documentation

- [Mapping package](../packages/savo_mapping.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Mapping operation](../operations/mapping_operation.md)
- [Locations plan](locations_test_plan.md)
