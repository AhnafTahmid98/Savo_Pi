# Robot Savo current system status

**Status date:** 2026-08-09

**Inspected artifact:** live Git checkout at `~/Savo_Pi`

**Scope:** Robot Savo ROS 2 repository and current workspace content

## Executive status

The repository is ready for the next gate: clean role-specific build/test on the intended Core and Edge targets followed by safe-idle and staged hardware regression. It is **not authorized for production motion**.

Phase 3 package documentation and Phase 4 architecture/hardware documentation are complete. The live checkout contains exactly 20 ROS package manifests and central pages, 18 architecture pages, and 11 hardware pages. The full-bringup, observer, and aggregate pre-real-test validators pass in the current development environment.

Motion remains blocked because the active geometry profile is provisional, production requires locked geometry, the D435 voxel profile defaults unvalidated, and current-source target/hardware regression is not recorded. An earlier Robot Savo baseline was exercised on physical hardware; that evidence does not replace regression of this source/configuration.

## Validation terminology

| Term | Meaning |
| --- | --- |
| Implemented | Required source/configuration/interface exists |
| Source-validated | Static validator or source-contract test passed |
| PC-validated | Build/tests ran successfully on a development PC |
| Target-validated | Build/tests ran on intended Core, Edge, or observer host |
| Hardware-validated | Behavior was exercised on physical hardware |
| Integration-validated | Multiple production components were exercised together |
| Blocked | An external dependency, measurement, environment, authorization, or earlier gate is missing |

## Validation executed for this status

| Check | Result | Finding |
| --- | --- | --- |
| Package coverage | PASS | 20 manifests and 20 corresponding central pages; no `savo_intent` page |
| Architecture/hardware coverage | PASS | Exactly 18 architecture and 11 hardware Markdown pages |
| Markdown relative links | PASS | No broken relative link in `README.md` or `docs/**/*.md` |
| `deploy/common/validate_full_bringup.sh` | PASS | Required launch/config/deploy contracts valid |
| `deploy/observer/validate_observer.sh` | PASS | Required assets present; observer remains read-only |
| `deploy/common/validate_pre_real_test_readiness.sh` | PASS | No failed aggregate checks |
| `git diff --check` | PASS | Documentation changes have no whitespace errors |

The aggregate validator rendered Core/Edge Netplan without applying it. Its optional schema-generator detail reported an environment block because system D-Bus is unavailable; this did not produce an aggregate failure. `rosdep check`, systemd render/verify, backup/restore, syntax/parsing, authority defaults, geometry/D435 gates, UI read-only integration, bridge boundary, and speech protocol-v2 checks passed.

## Authoritative deployment membership

Current `deploy/core/env_core.sh` and `deploy/edge/env_edge.sh` match the documented role sets:

- Core: 14 packages.
- Edge: 10 packages.
- `savo_observer`: workstation-only and absent from both production arrays.

`savo_ws/src/future/` contains staging content but no `package.xml`; it is not part of the 20-package workspace. The current working tree also contains user-owned, uncommitted `savo_ui` v2 source/config/test content. It is documented as an optional alternative; `ui_select.launch.py` still defaults to classic and Edge deploy still defaults UI off.

## Current authority and safety state

The current source confirms the fail-closed motion chain:

```text
approved command source
        -> savo_control
        -> /cmd_vel
        -> savo_perception command safety gate
        -> /cmd_vel_safe
        -> savo_base
        -> drivetrain hardware
```

- `savo_base` alone writes drivetrain outputs.
- `savo_perception` can independently stop/constrain motion.
- `savo_supervisor` grants/revokes permission; operation owners execute.
- `savo_bridge` exposes typed bounded operations only.
- `savo_ui` and `savo_observer` are read-only.
- Map and semantic-location release decisions remain operator-controlled.

## Implemented versus remaining validation

| Area | Current source state | Remaining gate |
| --- | --- | --- |
| Bringup | Distributed role/mode/profile graph source-validated | Clean target builds and two-Pi safe-idle |
| Geometry/TF | Xacro/profile/digest validators present | Measure, review, lock, regenerate, physically verify |
| Base/control/safety | C++ command and fail-closed execution paths present | Wheels-raised STOP/watchdog/polarity/sensor-gate regression |
| LiDAR/localization | Drivers, odometry, EKF, health present | Live rates/signs/TF/covariance/drift/stale validation |
| Mapping/locations | Autonomous workflow, persistence, quality/review/release present | Manual then guarded autonomous physical lifecycle and rollback |
| Navigation | Verified-map gateway/readiness/Nav2 integration present | Verified production map, tuning, recovery/cancel/routes |
| RealSense/VO/cloud | Acquisition/VO/filter source present | Live USB/RGB-D/VO; voxel cloud independently hardware-gated |
| Speech/UI | Bounded speech transport and read-only displays present | Edge audio/display/touch/freshness integration |
| Power | Role monitors/aggregate/shutdown request present | Live calibration, thresholds, fault/shutdown policy |
| Supervisor/bridge | Authority, persistence, credential/freshness gates present | Two-Pi revoke/recovery/socket/timeout integration |
| Observer | Read-only source validator passes | Build/connect on actual workstation/network |

## Motion blockers

Production motion remains blocked until applicable gates close:

1. Record exact revision and clean target dependency resolution/build/tests.
2. Lock measured geometry and verify generated footprint/fixed TF.
3. Pass Core and Edge safe-idle with control in `STOP`.
4. Validate motors, encoders, IMU, LiDAR, near-field safety, power, network, and time sync.
5. Validate supervisor arming, fault latch, authorization revocation, and shutdown.
6. Begin motion wheels-raised with physical emergency-stop control.
7. Complete manual mapping and verified production release before autonomous mapping or saved-map navigation.
8. Keep D435 voxel disabled until its separate real-hardware validation passes.

Two additional Phase 4 findings require closure during geometry and hardware integration:

- base/localization kinematics use `0.165 m` wheelbase and track while provisional URDF wheel centres imply `0.230 m` and `0.200 m`;
- base and head drivers separately initialize the same PCA9685 at Core bus 1 address `0x40`; channels do not overlap, but chip-wide initialization and concurrent access require validation.

## Source/documentation discrepancies requiring follow-up

- `savo_perception/config/topics.yaml` retains `savo_dashboard` in descriptive consumer metadata, although that package does not exist.
- Legacy `savo_ui/ui_mode_router_node.py` subscribes to `/savo_intent/intent_result`; no current package publishes it, and current C++ UI paths do not use it.
- A retained Python `savo_base` model docstring mentions `savo_intent`; production C++ base execution has no such dependency.

These references are outside the production role arrays and do not alter the current fail-closed command path. They were not silently changed during documentation work.

## Immediate next phase

On Core:

```bash
cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test
```

On Edge:

```bash
cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test
```

Then follow the [full robot test plan](../testing/full_robot_test_plan.md), beginning with safe-idle and no motion.

## Package documentation status

All 20 package pages, 18 architecture pages, and 11 hardware pages are indexed in [the documentation index](../README.md). Deployment and testing records remain subordinate to current source and require target/hardware evidence before production use.
