# Robot Savo Release Checklist

Use this checklist for a source release deployed to Core, Edge, or both. Complete the evidence fields rather than marking a release ready from memory.

## Release identity

- Release name/tag: ______________________________
- Git commit: ___________________________________
- Branch: _______________________________________
- Release date/timezone: _________________________
- Release owner: _________________________________
- Core revision: _________________________________
- Edge revision: _________________________________
- SavoMind image digest/version: _________________
- Geometry profile and digest: ___________________
- Active map release: ____________________________
- Location release: ______________________________

## Source hygiene

- Working tree is clean.
- `git diff --check` passes.
- No generated `build/`, `install/`, `log/`, cache, bag, model, database, map-session, or secret files are included.
- Package manifests and build/install rules are valid.
- Role package arrays match the intended deployment.
- Retired package names are absent from production arrays and launch files.
- Documentation and changelog describe the release-relevant changes.

## Static validation

Run from repository root:

```bash
bash deploy/common/validate_full_bringup.sh
bash deploy/observer/validate_observer.sh
bash deploy/common/validate_pre_real_test_readiness.sh

```

- Full-bringup validator: PASS.
- Observer validator: PASS.
- Aggregate validator has no FAIL result.
- Every BLOCKED result is listed below and has an owner.
Blocked items:

```text

```

## Dependencies

- Ubuntu/ROS target versions match the supported platform.
- Core dependency installer passes.
- Edge dependency installer passes.
- `rosdep` reports no unresolved role dependency.
- At least 8 GiB free space exists on each target before build/update.
- Edge speech/runtime group and directory policy are correct.

## Clean target builds

### Core

```bash
bash deploy/core/build_core.sh --clean --test

```

- All 14 Core packages are present.
- Build passes.
- Tests report zero failures and errors.
- `--allow-missing` was not used.
- Core build log retained.

### Edge

```bash
bash deploy/edge/build_edge.sh --clean --test

```

- All 10 Edge packages are present.
- Build passes.
- Tests report zero failures and errors.
- Edge build log retained.

### Observer

- Observer build/validation passes on the intended workstation.
- Observer remains read-only.

## Persistent state and rollback

- Current maps, locations, supervisor state, and required configuration are backed up.
- Backup integrity manifest verifies.
- Restore procedure has been tested for this release family.
- Current active install identity is recorded.
- Previous known-good install/release is retained.
- Rollback owner and command are recorded.
Rollback target:

```text

```

## Service configuration

- Rendered systemd units pass `systemd-analyze verify`.
- `SAVO_ROOT` and `SAVO_WS` point to the intended deployment.
- ROS domain and middleware match across Core, Edge, and observer.
- Only one service owns each role/component.
- Core defaults to `safe_idle` and `STOP`.
- Edge UI/speech/cloud feature flags match the validated scope.
- Mapping boot gate is disabled unless the release session explicitly requires it.
- Log rotation and storage permissions are correct.

## Safety gates

- Physical geometry is measured, reviewed, and locked for motion.
- Generated footprint and TF match the locked geometry.
- D435 obstacle cloud remains disabled unless its hardware gate passed.
- Supervisor startup, arming, revocation, fault latching, and shutdown are validated.
- Base watchdog and perception stale-input behavior are validated.
- No UI, observer, SavoMind, or bridge path bypasses Core authority.
- Operator-only map/location approval remains operator-only.

## Safe-idle deployment

- Core starts safely with no unintended motion.
- Core control reports `STOP`.
- Edge starts without duplicate bridge/UI/VO/RealSense owners.
- Core–Edge discovery and time synchronization are stable.
- Readiness and diagnostics are current.
- Services stop and restart cleanly.
- Boot behavior has been tested in a physically safe configuration.

## Regression scope

Mark every affected subsystem:

- Description/TF/geometry
- Base/motors/encoders
- Control/arbitration/recovery
- Perception/safety sensors
- LiDAR
- Localization/VO
- Mapping/release
- Navigation/recovery
- Locations
- Supervisor
- Head/camera/AprilTag
- RealSense/cloud
- Speech/SavoMind
- UI
- Bridge
- Power/shutdown
- Observer
- Deployment/network/storage
- Required component plans passed.
- Integration plans passed.
- Real-robot acceptance checklist passed for the approved scope.

## Final decision

DecisionSelectMeaningRelease approved[ ]All required gates passed for the stated scopeRelease conditionally approved[ ]Non-motion or restricted scope; every limitation documentedRelease blocked[ ]One or more required gates missingRelease rejected[ ]Failure or unacceptable risk requires correctionApproved scope and limitations:

```text

```

Sign-off:

```text
Release owner:
Safety/test operator:
Core deployment operator:
Edge deployment operator:
Reviewer:
Date/time:

```
