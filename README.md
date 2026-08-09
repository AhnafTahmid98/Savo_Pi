# Robot Savo

Robot Savo is a distributed ROS 2 indoor guide robot designed for safe campus navigation, autonomous mapping, speech interaction, semantic destinations, and operator-visible system state.

The robot uses two Raspberry Pi 5 computers with deliberately separated responsibilities:

* **`savo-core`** owns movement authority, motor execution, near-field safety, localization, mapping, navigation, semantic locations, head control, and global supervision.
* **`savo-edge`** owns RealSense acquisition, visual odometry, speech hardware, the touchscreen UI, the constrained SavoMind boundary, and edge-side power monitoring.

Both computers use this repository, but each one builds and runs only its approved package set. High-level AI, speech, UI, and observer components do not have direct motor authority.

> [!IMPORTANT]
> A successful source validation or ROS build is not permission to move the robot. Production startup defaults to `STOP`, physical geometry must be locked, safety inputs must be verified, and the staged real-robot test plan must pass before motion is authorized.

## System architecture

```text
User / operator / SavoMind
             │
             │ typed, bounded requests and observations
             ▼
          savo-edge
  speech · UI · bridge · VO · RealSense
             │
             │ ROS 2 over the dedicated Core–Edge link
             ▼
          savo-core
 supervisor · mapping · navigation · control
             │
             ▼
  perception safety gate → savo_base → motors
```

The motion path is intentionally fail-closed:

```text
Nav2 / approved teleoperation / authorized mission
                         │
                         ▼
                    savo_control
                         │
                         ▼
              savo_perception safety gate
                         │
                    /cmd_vel_safe
                         │
                         ▼
                     savo_base
                         │
                         ▼
                 drivetrain hardware
```

The following boundaries are non-negotiable:

* `savo_base` is the only package that executes drivetrain output.
* `savo_perception` can stop or constrain motion independently of AI behavior.
* `savo_supervisor` grants or revokes permission; it does not plan paths or drive motors.
* `savo_bridge` exposes a typed command boundary; it is not a generic ROS proxy.
* `savo_ui` and `savo_observer` are read-only presentation surfaces.
* SavoMind runs outside the ROS workspace and cannot bypass Core-side safety, readiness, or mission authority.

## Repository layout

```text
Savo_Pi/
├── README.md                  # repository entry point
├── CHANGELOG.md               # dated project changes
├── LICENSE                    # repository license
├── docs/                      # architecture, setup, deployment, and validation
├── deploy/                    # role builds, updates, services, networking, recovery
├── savo_ws/                   # ROS 2 Jazzy workspace
│   └── src/
│       ├── core/              # Core-owned runtime packages
│       ├── edge/              # Edge-owned runtime packages
│       └── shared/            # cross-role interfaces and infrastructure
└── tools/                     # approved development and diagnostic utilities
```

The workspace currently contains **20 ROS 2 packages**. The authoritative role membership is defined in:

* `deploy/core/env_core.sh`
* `deploy/edge/env_edge.sh`

See the [package ownership matrix](docs/packages/package_ownership_matrix.md) for the complete package list and runtime boundaries.

## Platform

The production target is:

| Component        | Target                                                            |
| ---------------- | ----------------------------------------------------------------- |
| Operating system | Ubuntu 24.04 LTS                                                  |
| ROS distribution | ROS 2 Jazzy                                                       |
| Core computer    | Raspberry Pi 5, hostname `core` or `savo-core`                    |
| Edge computer    | Raspberry Pi 5, hostname `edge` or `savo-edge`                    |
| Core–Edge link   | Dedicated Ethernet, default `192.168.50.1/24` ↔ `192.168.50.2/24` |
| Operator tools   | ROS 2 Jazzy workstation using `savo_observer` and RViz            |

## Validation before deployment

Run the source-level checks from the repository root:

```bash
cd ~/Savo_Pi

bash deploy/common/validate_full_bringup.sh
bash deploy/observer/validate_observer.sh
bash deploy/common/validate_pre_real_test_readiness.sh
```

The aggregate validator distinguishes:

* **PASS** — the checked source or contract is valid;
* **BLOCKED** — the check requires unavailable external tooling, target hardware, measurements, or Git metadata;
* **FAIL** — the repository violates a required contract.

For the complete hardware-free ROS regression on an Ubuntu 24.04 / ROS 2 Jazzy development computer:

```bash
cd ~/Savo_Pi
bash deploy/common/run_pre_real_test_regression.sh --clean-affected
```

## Role-specific builds

Build each target with its role script. These scripts verify the hostname and the complete required package set.

### Core

```bash
cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test
```

### Edge

```bash
cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test
```

The dependency installer uses the same role arrays as the build scripts:

```bash
bash deploy/common/install_role_deps.sh --role core
bash deploy/common/install_role_deps.sh --role edge
```

Do not build generated `build/`, `install/`, or `log/` directories into release archives. They are target-specific artifacts and are intentionally ignored by Git.

## Production startup

Prepare persistent Core runtime storage once before the first production launch:

```bash
cd ~/Savo_Pi
sudo bash deploy/core/prepare_runtime_storage.sh \
  --owner "$USER" \
  --group "$USER"
```

The role launchers start the distributed bringup in safe-idle mode and preserve `STOP` as the control default:

```bash
bash deploy/core/run_core.sh
bash deploy/edge/run_edge.sh
```

The production service units are rendered from the templates in `deploy/systemd/`. Do not install or enable a service until its role build, dependency check, runtime-directory preparation, and safe-idle verification have passed.

## Documentation

Start with:

1. [Documentation index](docs/README.md)
2. [Fresh installation and commissioning](docs/setup/README.md)
3. [Current system status](docs/status/current_system_status.md)
4. [Package ownership matrix](docs/packages/package_ownership_matrix.md)
5. [System overview](docs/architecture/system_overview.md)
6. [Two-Pi architecture](docs/architecture/two_pi_architecture.md)
7. [Production operations](docs/operations/README.md)
8. [Operator quick start](docs/operations/operator_quick_start.md)
9. [Emergency stop and recovery](docs/operations/emergency_stop_and_recovery.md)
10. [Full robot test plan](docs/testing/full_robot_test_plan.md)
11. [Requirements traceability](docs/testing/requirements_traceability_matrix.md)
12. [Regression matrix](docs/testing/regression_matrix.md)
13. [Final documentation audit](docs/audits/final_documentation_audit_2026-08-09.md)

Package-local `README.md` files document implementation details. Cross-package documents in `docs/` describe system contracts, deployment, and validation. Where a dated audit and the current source disagree, the current source and its validators take precedence.

## Development rules

* Keep production motion paths in C++ unless an approved exception is documented.
* Keep Python fallbacks and diagnostics separate from the production authority path.
* Add new package membership through the role environment arrays; do not duplicate package lists in deployment scripts.
* Treat measurements, calibration values, hardware IDs, and validation results as controlled engineering data.
* Never document a command as validated without recording where it ran and what evidence was produced.
* Do not place secrets, model files, runtime databases, maps, ROS bags, or generated build artifacts in Git.
* Preserve fail-closed defaults when adding launch arguments, services, actions, or remote interfaces.

## Project status

The source tree has completed its pre-real-test closure and the Phase 3–7 documentation system: all 20 ROS 2 packages have central source-backed pages, all applicable subsystems have stable-ID test plans, and requirements/evidence/regression governance is in place. The current checkout still requires clean unrestricted target builds/tests, locked physical geometry, and staged hardware/integration acceptance before production motion.

See [Current system status](docs/status/current_system_status.md) for the evidence, blockers, and next gate.

## License

Copyright © 2026 Ahnaf Tahmid. All rights reserved.

This repository is proprietary unless an individual file explicitly states otherwise. See [LICENSE](LICENSE).
