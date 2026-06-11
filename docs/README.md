# Robot Savo Documentation

Robot Savo is an AI-assisted autonomous indoor guide robot for campus navigation, voice interaction, mapping, and safe human-friendly movement.

This documentation explains the robot architecture, ROS 2 package layout, hardware structure, setup process, deployment workflow, and testing plan.

## Project structure

Robot Savo uses one ROS 2 repository for both Raspberry Pi units:

* `savo-core` — movement authority, base control, safety, localization, mapping, and navigation
* `savo-edge` — RealSense/depth, visual odometry, speech/audio, UI, and Dockerized AI helper services from the separate `Robot_Savo_Server` repo

The ROS 2 workspace is organized by package role:

```text
Savo_Pi/savo_ws/src/
├── core/      # packages mainly for savo-core
├── edge/      # packages mainly for savo-edge
├── shared/    # packages used by both Pis
└── future/    # planned or optional packages
```

Each Pi pulls the same `Savo_Pi` ROS 2 repository, but builds and runs only the packages required for its role.

## Repositories

Robot Savo uses two main repositories:

| Repository            | Role                                                                                                                                                |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Savo_Pi`             | ROS 2 workspace for robot packages, launch files, hardware control, perception, localization, mapping, navigation, UI, and speech integration       |
| `Robot_Savo_Server`   | Dockerized AI/helper server running on `savo-edge`, used for LLM/intent/helper services                                                             |

`Robot_Savo_Server` is separate from the ROS 2 workspace. It supports the robot, but it does not directly control the motors.

## Documentation map

| Section                          | What it covers                                                                                                                                     |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`architecture/`](architecture/) | System design, two-Pi architecture, safety model, perception, localization, mapping/navigation, speech flow, networking, and ROS 2 topic contracts |
| [`packages/`](packages/)         | ROS 2 package responsibilities and ownership between `savo-core`, `savo-edge`, and shared packages                                                 |
| [`hardware/`](hardware/)         | Robot layer layout, wiring, GPIO/I2C map, power, sensor mounting, mechanical notes, and measurement checklist                                      |
| [`setup/`](setup/)               | Setup guides for `savo-core`, `savo-edge`, Ethernet, ROS networking, RealSense, audio, Tailscale, and time sync                                    |
| [`deployment/`](deployment/)     | Role-based builds, Docker services, systemd services, and production startup                                                                       |
| [`testing/`](testing/)           | Component validation overview, per-unit hardware validation tables, and test plans for base, control, perception, localization, description, mapping, navigation, speech, VO, and full robot validation |
| [`diagrams/`](diagrams/)         | Draw.io diagrams for compute split, ROS graph, networking, safety, perception, and navigation pipelines                                            |
| [`assets/`](assets/)             | Screenshots, RViz views, TF trees, test proof, and documentation images                                                                            |

## Read first

Start with these documents:

1. [`architecture/system_overview.md`](architecture/system_overview.md)
2. [`architecture/two_pi_architecture.md`](architecture/two_pi_architecture.md)
3. [`packages/package_ownership_matrix.md`](packages/package_ownership_matrix.md)
4. [`hardware/robot_layer_layout.md`](hardware/robot_layer_layout.md)
5. [`hardware/measurement_checklist.md`](hardware/measurement_checklist.md)
6. [`deployment/role_based_builds.md`](deployment/role_based_builds.md)
7. [`setup/ups_hat_setup.md`](setup/ups_hat_setup.md)
8. [`testing/component_validation_overview.md`](testing/component_validation_overview.md)

## Current engineering direction

Robot Savo is built around a two-Pi architecture:

| Unit        | Main responsibility                                                                                                     |
| ----------- | ----------------------------------------------------------------------------------------------------------------------- |
| `savo-core` | Owns movement authority, motor execution, safety decisions, localization, mapping, and Nav2                             |
| `savo-edge` | Handles RealSense/depth, visual odometry, speech/audio, UI, and Dockerized AI helper services from `Robot_Savo_Server`  |

Motion authority stays on `savo-core`. Edge-side systems can provide perception, speech, VO, or AI support, but they do not directly control the motors.

## Core design rules

* Safety is independent from AI.
* Motor execution is owned by `savo_base`.
* Velocity commands pass through safety before reaching the base driver.
* `savo_description` is the shared physical truth for URDF, TF frames, RViz, mapping, navigation, and 3D costmap alignment.
* `savo_perception` is shared because perception runs on both sides: safety/range sensing on `savo-core`, and depth/RealSense processing on `savo-edge`.
* Packages are built role-by-role, not all together on both Pis.

## Documentation status

These docs are written as the robot develops. Some files start as placeholders and are completed after real measurements, package validation, RViz screenshots, hardware tests, and deployment checks.

Use `docs/assets/` for proof and screenshots, such as:

```text
docs/assets/
├── savo_description/
├── testing/
└── screenshots/
```

Do not treat untested commands, guessed measurements, or placeholder diagrams as final documentation.
