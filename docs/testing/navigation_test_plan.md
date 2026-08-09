# Navigation Test Plan

## Objective

Verify production saved-map navigation and supported live-mapping development mode, release/readiness/authority admission, public gateway isolation, named locations, cancellation/recovery, Nav2 profiles, and safe command routing.

## Scope

Nav2 lifecycle, active release verification, readiness/map context, public versus internal actions, pose/named goals, cancellation, stale/mismatch/authority denial, behavior trees, control recovery, LiDAR costmaps, optional D435 voxel profile, restart and negative release tests.

## Test ownership

Navigation maintainer owns NAV-001–008; navigation operator/safety reviewer owns NAV-009–014.

## Safety classification

NAV-001–008 are `STATIC`, `UNIT`, `SOURCE-CONTRACT`, `PC`, `TARGET-NON-HARDWARE`, `FAULT-INJECTION`, or `PERSISTENT-STATE` / `NO-MOTION`. NAV-009–014 are `HARDWARE-ACTUATING`, `INTEGRATION`, `RECOVERY`, or `ACCEPTANCE` / `GUARDED-FLOOR-MOTION`.

## Preconditions

Locked geometry and verified production map/release for saved-map operation; base/control/perception/LiDAR/localization/supervisor/mapping/locations relevant gates PASS; clear route/test zone/E-stop team; LiDAR-only profile first; D435 voxel remains disabled until its own gate passes.

## Required hardware

Complete motion/safety/localization/LiDAR system; D435 only for optional voxel scope; controlled routes/obstacles and destination references.

## Required software / configuration

Nav2, production/development/live launches, active-map/readiness/gateway/location/recovery/costmap/BT configs, map/location release contracts and `savo_msgs` actions.

## Interfaces under test

Public guarded navigation/named-location/coverage interfaces and configured state/readiness/status; internal Nav2 actions remain isolated. Output routes through `/cmd_vel_nav -> savo_control -> /cmd_vel -> perception -> /cmd_vel_safe`.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| NAV-001 | T0 `STATIC` | Inventory production, saved-map development and live-mapping launches; verify public gateway/internal action separation, BT files fixed by config, active map/release/geometry/location checks, TF owners and command routing. |
| NAV-002 | T1 `UNIT`/`PC` | Build/tests pass for map context/digests, readiness/dependency freshness, goal validator/admission/arbiter/gateway/cancel, location navigation, recovery, coverage proxy/path and launch/profile/BT/costmap contracts. |
| NAV-003 | T2 `TARGET-NON-HARDWARE` | Launch argument/config parsing and lifecycle with fixture map; production requires an active verified contract, while development saved-map launch is clearly non-production. |
| NAV-004 | T2 `SOURCE-CONTRACT` | Valid goal reaches only the guarded public gateway; invalid frame/pose/map/release/authority/mode/safety/stale/busy/recovery-conflict and external BT-path requests reject with reason. |
| NAV-005 | T2 `SOURCE-CONTRACT` | Named lookup accepts only approved enabled location with matching active map/location release and uses approved approach pose; candidate/pending/disabled/ambiguous/mismatch rejects. |
| NAV-006 | T6 `FAULT-INJECTION` | Simulate stale readiness, backend loss, feedback timeout, authority revoke, map change and cancel; backend cancellation is bounded/acknowledged and no command remains active. |
| NAV-007 | T6 `PERSISTENT-STATE` | Negative active-release cases (missing/corrupt/hash/geometry/map/location mismatch) prevent lifecycle/admission; development launch must not satisfy production evidence. |
| NAV-008 | T7 `RECOVERY` | Restart gateway/Nav2/map server/AMCL; lifecycle returns only with verified context/fresh evidence, active goals do not replay, duplicate owners do not appear. |
| NAV-009 | T3 `HARDWARE-NON-ACTUATING` | Stationary LiDAR-only production profile: verify map/AMCL/TF/costmaps/readiness and one `map -> odom`; no motion goal submitted. |
| NAV-010 | T4/T5 `HARDWARE-ACTUATING` | Guarded short pose goals at reduced approved limits; verify planner/controller, command path, costmap obstacles, arrival behavior and measured tolerance. Missing formal tolerance keeps acceptance BLOCKED. |
| NAV-011 | T5 `HARDWARE-ACTUATING` | Validate cancel at planned/executing states, safety stop, authority revoke and backend loss; zero motion and terminal correlated result are required. |
| NAV-012 | T5 `HARDWARE-ACTUATING` | Exercise configured recovery under guarded conditions; recovery count/behavior must follow active config, stay gated, and stop on repeated failure. |
| NAV-013 | T5 `INTEGRATION` | Navigate to an approved named location under matching releases; verify lookup, goal admission, approach pose and optional arrival/tag confirmation without granting approval authority. |
| NAV-014 | T5 `ACCEPTANCE` | Only after D435 cloud plan PASS, explicitly test voxel profile for marking/clearing/self/floor/stale/performance. Failure disables voxel and returns to validated LiDAR-only profile. |

## Pass criteria

Only verified context and fresh authority admit goals; internal actions stay isolated; commands traverse all gates; cancel/revoke/recovery are bounded; named goals match releases; LiDAR profile works before optional voxel acceptance.

## Blocked criteria

No verified production map/location release, unlocked geometry, incomplete upstream hardware gates, no approved route/tolerance/recovery criterion, or D435 cloud not accepted.

## Failure criteria

Admission bypass, wrong map/location/BT/frame accepted, direct command bypass, stale readiness, continued motion after cancel/revoke, unsafe recovery, duplicate TF/lifecycle owner, or unvalidated voxel enabled.

## Abort criteria

E-stop/cancel on person/obstacle hazard, localization/TF/readiness loss, route divergence, command/source mismatch, repeated recovery, collision, inability to stop, or map/release mismatch.

## Evidence to retain

Commit/profiles/geometry/map/location release/digests, lifecycle/readiness/TF/costmaps, goals/correlation/feedback/results, command path and cancel/recovery timing, route/arrival measurements/video, voxel bags/performance if applicable.

## Regression triggers

Nav source/params/BTs/gateway/admission/readiness/recovery; map/location/release/digest; geometry/footprint/TF; LiDAR/costmaps; D435 cloud/voxel; control/perception/supervisor/interfaces.

## Current validation status

Extensive source tests exist. Current verified production map, target Nav2 build, AMCL/tuning, physical goals/cancel/recovery/named routes and optional voxel acceptance remain NOT RUN/BLOCKED by prerequisites.

## Related documentation

- [Navigation package](../packages/savo_nav.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Navigation operation](../operations/navigation_operation.md)
- [Mapping plan](mapping_test_plan.md)
