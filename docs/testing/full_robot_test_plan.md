# Full Robot Staged Test Plan

## Purpose and authority

This is the dependency-ordered integration sequence for Robot Savo. It does not replace component plans or authorize motion. A `FAIL` or unresolved prerequisite `BLOCKED` prevents every dependent later stage. Use the exact result vocabulary and evidence rules in [test evidence guidelines](test_evidence_guidelines.md).

Physical procedures are written here but were not executed during the Phase 7 documentation audit.

## Test ownership and safety

The test lead owns the run record; package owners own component results; an emergency-stop operator and independent reviewer are mandatory for actuating stages. Follow [universal failure and abort criteria](failure_and_abort_criteria.md). `SYS-001`–`SYS-006` are `NO-MOTION`; `SYS-007` begins `WHEELS-RAISED`; `SYS-008` onward may use `GUARDED-FLOOR-MOTION` only after explicit reauthorization.

## Staged sequence

| Test ID | Gate | Required prerequisite evidence | Minimum procedure / acceptance | Later scope blocked on FAIL/BLOCKED |
| --- | --- | --- | --- | --- |
| SYS-001 | Source/static validation | Exact commit/worktree | Run the three repository validators, package/interface/link/command audits and `git diff --check`; no repository FAIL or unexplained mismatch | All target work |
| SYS-002 | Clean PC/role builds | SYS-001 PASS; ROS 2 Jazzy environment | Run hardware-free regression where supported, then clean Core/Edge/observer builds/tests; retain `colcon test-result --verbose` | Target commissioning |
| SYS-003 | Fresh-install commissioning | SYS-002 PASS; target identity | Follow setup checklists for OS/ROS/dependencies/permissions/network/time/storage/systemd without starting motion | Distributed runtime |
| SYS-004 | Safe-idle distributed bringup | BRG-001–010 applicable PASS; Core STOP | Start Core then Edge with approved safe-idle/profile; one owner per component; readiness/heartbeat/stale/shutdown/restart evidence | Live hardware tests |
| SYS-005 | Non-actuating hardware | Description measurement process and electrical inspection | Run DSC, LID, LOC stationary, RLS, PWR, HED camera, SPH devices, UI and observer non-actuating cases; lock geometry only after review | Actuator tests |
| SYS-006 | Safety/authority non-motion integration | SYS-005 applicable PASS | Run CTL/PER/BAS dry-run, SUP/BRD/locations persistence/fault cases; prove STOP, stale, authority, socket, backup and release rejection | Motion authority |
| SYS-007 | Wheels-raised base/control/perception | Locked geometry; E-stop/stands/operators; BAS/CTL/PER prerequisites PASS | Execute BAS-006/007, CTL-009 and PER-005 one bounded direction/case at a time; include encoder signs, STOP/watchdog and publisher loss | Floor motion |
| SYS-008 | Guarded floor/localization | SYS-007 PASS; clear course | Execute BAS-008, CTL-010, PER-006 and LOC-006; measure direction, stop behavior, scale/drift and safety envelope; missing approved thresholds keep acceptance BLOCKED | Mapping/nav |
| SYS-009 | Manual mapping and map verification | SYS-008 and LiDAR/localization PASS | Execute MAP-008–010: manual mapping first, save, verify artifacts/hash/quality; do not promote automatically | Release/nav/autonomous mapping |
| SYS-010 | Operator map/location release | Verified map, quality and reviewer | Execute MAP-013/014 and LCT-007/010 as applicable; explicit operator review; immutable map/location/geometry context and rollback evidence | Production navigation |
| SYS-011 | LiDAR-only navigation and named locations | SYS-010 PASS; supervisor ready | Execute NAV-009–013 using reduced guarded limits; validate goal admission, routes, cancel/revoke/recovery and named destinations | Optional voxel/acceptance |
| SYS-012 | Autonomous mapping | Manual workflow and guarded nav/control behavior proven | Execute MAP-011/012 and fault/cancel cases in controlled area; semantic candidates remain operator-reviewed | System interaction acceptance |
| SYS-013 | Edge/SavoMind interaction | Speech/UI/bridge component cases PASS | Execute SPH-006–010, UI-006/007 and BRD-007–009; typed operations remain subject to Core readiness/authority | Full integration |
| SYS-014 | Optional D435 voxel | RLS/VO stream evidence, PER-008 reviewed, LiDAR-only nav PASS | Execute PER-009/010 and NAV-014; self/floor/freshness/performance/costmap evidence. On failure disable voxel and retain LiDAR-only profile | Voxel release only |
| SYS-015 | Failure/recovery/rollback | All affected normal paths PASS | Exercise approved non-destructive failure, cancellation, restart, backup/restore and rollback cases; no unsafe replay; incidents resolved | Acceptance |
| SYS-016 | Real-robot acceptance | All requested-scope component/system IDs PASS; blockers/deviations closed/approved | Complete the [acceptance checklist](real_robot_acceptance_checklist.md), record operating envelope and explicit return-to-service authority | Production operation |

## Hardware-free regression gate

The repository script is hardware-free: it validates source, checks dependencies, builds/tests its explicit affected-package list, generates launch arguments, and runs readiness validation. It never launches robot nodes or publishes commands.

```bash
cd ~/Savo_Pi
bash deploy/common/run_pre_real_test_regression.sh --clean-affected
```

Run it only on a supported ROS 2 Jazzy PC after reviewing its current source. Its affected-package list is not all 20 packages and therefore does not replace clean role builds. A physical-prerequisite BLOCKED result may remain, but any repository FAIL blocks SYS-003.

## Stage evidence and release decision

Each SYS result links the underlying component IDs, commit/hosts/config/hardware/geometry/map/location identity, raw artifacts, measurements, deviations and reviewer. A narrower scope may mark unrelated optional stages N/A with rationale; no motion-related prerequisite may be waived silently.

Successful documentation, source validation or safe-idle bringup is not production acceptance. SYS-016 alone records the requested operating envelope and return-to-service decision.
