# Robot Savo Current System Status

**Status date:** 2026-08-06
**Inspected artifact:** `Savo_Pi_2026-08-06_01-12-38(1).zip`
**Scope:** `Savo_Pi` ROS 2 repository only

## Executive status

The current source snapshot is suitable for the next validation stage: **clean role-specific build and test on the Core and Edge target environments**.

It is **not authorized for production motion**.

The repository passes the dedicated full-bringup and observer source validators. The aggregate pre-real-test validator reports `BLOCKED`, not `FAIL`. The two current environment blockers are:

1. Git metadata is absent from the exported ZIP, so `git diff --check` cannot run against the live checkout.
2. `rosdep` is unavailable in the inspection environment, so target dependency resolution cannot be verified here.

Independent motion gates remain closed because:

* the active geometry profile is marked `measurement_state: provisional`;
* production launch requires locked geometry by default;
* the D435 voxel path defaults to unvalidated and disabled;
* clean Core and Edge role builds/tests are not recorded in this exported snapshot;
* staged safe-idle, sensor, actuator, authority, mapping, and navigation validation is still required for the current source revision.

An earlier Robot Savo snapshot was exercised on the physical robot. That history is useful baseline evidence, but it does not replace regression testing of this newer source snapshot.

## Status interpretation

| Level                        | Meaning in this report                                                       |
| ---------------------------- | ---------------------------------------------------------------------------- |
| Implemented                  | The required source/configuration/interface exists in the inspected snapshot |
| Source-validated             | Static validators or source-contract checks passed                           |
| PC-validated                 | A retained log or dated audit records successful development-PC build/tests  |
| Target validation required   | The current Core Pi, Edge Pi, or observer host must build/test the source    |
| Hardware validation required | The current source must be exercised through the staged physical procedure   |
| Blocked for motion           | A fail-closed prerequisite prevents motion authorization                     |

## Validation executed during this inspection

### Full distributed bringup validator

```bash
bash deploy/common/validate_full_bringup.sh
```

**Result:** `PASS`

The validator confirmed required bringup files, launch/config parsing, role scripts, retired-package exclusion from deployment arrays, production map paths, AM-8 release wiring, and absence of `/tmp` production-state defaults.

### Observer validator

```bash
bash deploy/observer/validate_observer.sh
```

**Result:** `PASS`

The observer source validation confirms that the desktop/browser observer remains read-only and that its required source assets are present.

### Aggregate pre-real-test validator

```bash
bash deploy/common/validate_pre_real_test_readiness.sh
```

**Result:** `BLOCKED` with no failed checks

| Check                        | Result  | Inspection finding                                                              |
| ---------------------------- | ------- | ------------------------------------------------------------------------------- |
| Required files               | PASS    | All required files are non-empty                                                |
| Git whitespace check         | BLOCKED | Exported ZIP has no `.git` metadata                                             |
| Runtime zero-byte policy     | PASS    | No empty runtime-required files; 22 intentional ROS/package marker files        |
| Launch executable references | PASS    | No removed or empty localization executables referenced                         |
| YAML parsing                 | PASS    | All non-empty YAML parsed                                                       |
| XML parsing                  | PASS    | All non-empty XML parsed                                                        |
| Python parsing               | PASS    | All non-empty Python parsed                                                     |
| Shell syntax                 | PASS    | 40 shell scripts passed `bash -n`                                               |
| Netplan render               | PASS    | Core and Edge templates rendered without applying changes                       |
| Systemd render/verify        | PASS    | Units rendered and passed `systemd-analyze verify`                              |
| Backup/restore               | PASS    | Integrity, restoration, and overwrite refusal passed                            |
| `rosdep check`               | BLOCKED | `rosdep` unavailable in this environment                                        |
| Observer read-only boundary  | PASS    | Observer source validator passed                                                |
| Control startup              | PASS    | Deployment default remains `STOP`                                               |
| Geometry gate                | PASS    | Validator truthfully reports `BLOCKED_FOR_MOTION: geometry_not_locked`          |
| D435 voxel default           | PASS    | Hardware-validation flag defaults to `false`                                    |
| AM-8 release requirements    | PASS    | Quality approval, review gateway, and contract v2 are present                   |
| Diagnostic motion safety     | PASS    | Moving diagnostics require approved ROS paths and explicit physical opt-in      |
| UI authority                 | PASS    | Mapping, location, speech, and system feeds remain read-only                    |
| Bridge boundary              | PASS    | Typed STOP/teleop/navigation/mapping/query boundary; operator approval excluded |
| Speech transport v2          | PASS    | Bounded authenticated transport and playback acknowledgement contract present   |

## Retained validation evidence

The repository contains the following PC-validation logs:

| Package             | Evidence                                                       | Recorded result                                                                              |
| ------------------- | -------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `savo_base`         | `docs/logs/pc_validation/savo_base_colcon_test_pc.txt`         | Package test run passed; retained summary reports 403 tests, 0 errors, 0 failures, 0 skipped |
| `savo_localization` | `docs/logs/pc_validation/savo_localization_colcon_test_pc.txt` | Package test run passed; retained summary reports 403 tests, 0 errors, 0 failures, 0 skipped |
| `savo_realsense`    | `docs/logs/pc_validation/savo_realsense_pc_validation.txt`     | Build and tests passed; 86 tests, 0 errors, 0 failures, 0 skipped                            |

The dated pre-real-test completion audit additionally records:

* 25 focused Robot source-contract tests passed;
* 362 SavoMind regression tests passed;
* a cross-repository speech protocol v2 smoke test passed;
* backup/restore, systemd rendering, Netplan rendering, and strict C++ checks passed in that audit environment.

These are historical snapshot records. Re-run the relevant suites after source changes and on the intended target environment.

## Subsystem status

| Subsystem           | Current source state                                                                                             | Current validation boundary                                                               | Next required gate                                                                             |
| ------------------- | ---------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| Distributed bringup | Implemented and source-validated                                                                                 | Role-selecting Core/Edge orchestration, readiness aggregation, safe defaults, AM-8 wiring | Clean target builds; safe-idle launch on both Pis                                              |
| Base execution      | Production C++ drivetrain path present; Python fallback/diagnostics separated                                    | PC test evidence retained; earlier hardware baseline exists                               | Wheels-raised regression, watchdog, direction, STOP, and stale-command tests on current source |
| Control             | C++ control/mode/recovery implementation present                                                                 | Source package and extensive tests present; current target run not retained here          | Target build/test, command arbitration, shaping, mode, and recovery integration                |
| Near-field safety   | C++ ToF/ultrasonic fusion and `/cmd_vel_safe` gate present                                                       | Source contracts pass; hardware thresholds require staged confirmation                    | Live sensor health, stop/slow zones, stale input, and command-gate tests                       |
| LiDAR               | RPLIDAR ownership, scan, filtering, health, and watchdog source present                                          | Package README validation status is stale and requires reconciliation                     | Target build plus live A1 scan/rate/frame/mapping-ready verification                           |
| Localization        | C++ IMU, encoder odometry, EKF bringup, and health source present                                                | Retained PC tests pass; current full robot integration not established                    | Live IMU/encoder sign, TF, covariance, EKF, drift, and stale-input validation                  |
| Robot description   | URDF/Xacro, frames, footprint generation, and geometry validator present                                         | Source parses; geometry profile is explicitly provisional                                 | Physically measure, review, lock, regenerate, and verify TF/footprint                          |
| Mapping             | Production C++ workflow, exploration, coverage, saving, quality, semantic, and release source present            | Source-level architecture complete; strategy tuning and real mission evidence pending     | Manual map, save/verify, guarded autonomous mission, review, and AM-8 release                  |
| Navigation          | Nav2 orchestration, readiness, goal admission, production release verification, and recovery integration present | Source marked complete; physical tuning pending                                           | Navigate only from a verified production map after localization/safety gates pass              |
| Semantic locations  | Persistent registry, review lifecycle, typed services, and release source present                                | Source implementation present; full physical lifecycle not recorded here                  | Candidate capture, operator review, release, restart persistence, and named navigation         |
| Supervisor          | Readiness, mission authority, arming, latching, persistence, and shutdown source present                         | Source and runtime fixtures present; package requires staged physical validation          | Core/Edge startup, arm/disarm, fault latch, recovery, authority revoke/resume tests            |
| Head and Pi Camera  | Pan/tilt, scan, head TF, camera and AprilTag source present                                                      | Package notes validate GStreamer `libcamerasrc`; other camera paths are not approved      | Current-source pan/tilt limits, camera stream, TF, scan, and AprilTag integration              |
| RealSense           | D435 ownership, stream monitoring, health, and depth extraction present                                          | Retained PC validation passed                                                             | Live USB3, stream rate, frame, restart, depth, and thermal/bandwidth tests on Edge             |
| Visual odometry     | C++ RGB-D odometry, health, diagnostics, and republisher present                                                 | Standalone source/test structure present; EKF fusion intentionally gated                  | Stable live `/vo/odom`, covariance, dropout/recovery, then guarded EKF integration             |
| D435 obstacle cloud | Filtered obstacle-cloud source and Nav2 companion profile present                                                | Synthetic/source validation only                                                          | Measure self-filter bounds and validate live cloud before enabling voxel profile               |
| Speech              | C++ audio runtime and protocol v2 SavoMind round trip present                                                    | Source contract and dated cross-repository smoke evidence                                 | ReSpeaker/speaker device, wake/VAD, playback, gating, cancel, recovery, and latency tests      |
| UI                  | Read-only C++ framebuffer UI and live state subscriptions present                                                | Source integration passes; camera preview disabled by default                             | 800×480 display/touch/freshness/safety-overlay validation on Edge                              |
| SavoMind bridge     | Typed bounded observation and command adapters present                                                           | Source boundary validation passes; no generic authority exposed                           | Live Edge socket ownership, peer credentials, timeout, stale-state, and command tests          |
| Power               | Core UPS, Edge UPS, base battery, aggregate state, health, and shutdown source present                           | Source and tests present; percentage validity remains calibration-dependent               | Live readings, calibration validity, low-power policy, fault, and shutdown validation          |
| Observer            | Browser/RViz/telemetry source present and source-validated                                                       | Read-only validator passes                                                                | Build and connect from the actual operator workstation over the production network             |
| Deployment/recovery | Role installers/builds, staged updates, services, storage, backups, and network renderers present                | Static validation and backup/restore test pass                                            | Run dependency resolution, role build/test, service install, restart, and rollback on targets  |

## Motion blockers

Production motion must remain blocked until all applicable items are closed:

1. The live Git checkout passes `git diff --check` and identifies the exact revision under test.
2. `rosdep` resolves all selected role dependencies on the target operating system.
3. Core and Edge complete clean role-specific builds and tests.
4. The physical geometry profile changes from provisional to measured/reviewed/locked.
5. Generated footprint and fixed TF are regenerated from the locked profile and verified.
6. Both Pis pass safe-idle bringup with Core control in `STOP`.
7. Motor, encoder, IMU, LiDAR, near-field sensors, power, network, and time synchronization pass their component stages.
8. Supervisor arming, fault latching, authorization revocation, and controlled shutdown pass.
9. Motion begins with wheels raised and an operator at the physical emergency stop.
10. Manual mapping and production map verification pass before autonomous mapping or saved-map navigation.
11. The D435 VoxelLayer remains disabled until its independent hardware-validation procedure passes.

## Immediate next phase

Run Phase 1 target builds without launching motion:

### Core Pi

```bash
cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test
```

Expected gate:

* all 14 Core role packages are present;
* clean build completes;
* selected package tests complete with zero failures;
* no robot launch is started by the build script.

### Edge Pi

```bash
cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test
```

Expected gate:

* all 10 Edge role packages are present;
* clean build completes;
* selected package tests complete with zero failures;
* no camera, audio, display, bridge, or robot launch is started by the build script.

After both build gates pass, continue with the safe-idle target and hardware stages in `docs/testing/full_robot_test_plan.md`.

## Documentation debt identified in Phase 1

The package list and deployment matrix are now corrected. Several older detailed documents still contain stale planning-era content, including references to retired package names or incomplete package status. Those files must be reconciled during the architecture, package, deployment, and test documentation phases before they are treated as production procedures.
