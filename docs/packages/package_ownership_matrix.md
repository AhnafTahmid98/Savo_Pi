# Robot Savo Package Ownership Matrix

**Snapshot date:** 2026-08-09
**Workspace:** ROS 2 Jazzy
**Package count:** 20

This document defines package placement, deployment membership, primary runtime host, and authority boundaries for the current `Savo_Pi` workspace.

The authoritative build membership is implemented in:

* `deploy/core/env_core.sh`
* `deploy/edge/env_edge.sh`

Package manifests, launch files, CMake/setup files, and runtime code define the implementation. This matrix is the cross-package reference for maintainers and deployment tooling.

## Source layout

```text
savo_ws/src/
├── core/
│   ├── savo_base
│   ├── savo_control
│   ├── savo_head
│   ├── savo_lidar
│   ├── savo_localization
│   ├── savo_locations
│   ├── savo_mapping
│   └── savo_nav
├── edge/
│   ├── savo_realsense
│   ├── savo_speech
│   ├── savo_ui
│   └── savo_vo
└── shared/
    ├── savo_bridge
    ├── savo_bringup
    ├── savo_description
    ├── savo_msgs
    ├── savo_observer
    ├── savo_perception
    ├── savo_power
    └── savo_supervisor
```

Folder placement describes ownership and reuse; it does not by itself mean that a package runs on both Pis. Runtime membership is controlled by the role arrays and launch arguments.

## Deployment sets

### Core build set

`SAVO_CORE_BUILD_PACKAGES` contains 14 packages:

```text
savo_msgs
savo_description
savo_bringup
savo_perception
savo_power
savo_supervisor
savo_base
savo_control
savo_lidar
savo_localization
savo_locations
savo_mapping
savo_nav
savo_head
```

### Edge build set

`SAVO_EDGE_BUILD_PACKAGES` contains 10 packages:

```text
savo_msgs
savo_description
savo_bringup
savo_bridge
savo_perception
savo_power
savo_realsense
savo_speech
savo_ui
savo_vo
```

### Observer build set

The observer workstation builds through:

```bash
colcon build --packages-up-to savo_observer --symlink-install
```

`savo_observer` is intentionally not part of the Core or Edge production role arrays. It is a read-only operator surface.

## Complete ownership matrix

| Package                                                                   | Version | Source folder | Production deployment               | Primary ownership                                                                                                                                           |
| ------------------------------------------------------------------------- | ------: | ------------- | ----------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`savo_base`](../../savo_ws/src/core/savo_base/README.md)                 |   0.1.0 | `core/`       | Core                                | Final drivetrain execution, mecanum mixing, motor-board access, command watchdog, base state, and base diagnostics                                          |
| [`savo_control`](../../savo_ws/src/core/savo_control/README.md)           |   0.1.0 | `core/`       | Core                                | Control mode, command selection/shaping, bounded rotation and approach control, stuck detection, and recovery coordination                                  |
| [`savo_head`](../../savo_ws/src/core/savo_head/README.md)                 |   0.1.0 | `core/`       | Core                                | Pan/tilt actuation, Pi Camera stream ownership, head scan behavior, dynamic head TF, and AprilTag confirmation                                              |
| [`savo_lidar`](../../savo_ws/src/core/savo_lidar/README.md)               |   0.1.0 | `core/`       | Core                                | RPLIDAR A1 serial ownership, `/scan` production, filtering, watchdogs, health, and mapping/navigation-ready scan output                                     |
| [`savo_localization`](../../savo_ws/src/core/savo_localization/README.md) |   0.1.0 | `core/`       | Core                                | BNO055 IMU, four-wheel encoder odometry, EKF integration, localization state, and localization health                                                       |
| [`savo_locations`](../../savo_ws/src/core/savo_locations/README.md)       |  0.14.0 | `core/`       | Core                                | Authoritative semantic-location registry, candidate lifecycle, SQLite persistence, release identity, and typed location services                            |
| [`savo_mapping`](../../savo_ws/src/core/savo_mapping/README.md)           |   0.1.0 | `core/`       | Core                                | Mapping modes, SLAM workflow, exploration/coverage orchestration, map saving, quality evaluation, semantic registration, and production-map release         |
| [`savo_nav`](../../savo_ws/src/core/savo_nav/README.md)                   |   0.1.0 | `core/`       | Core                                | Nav2 orchestration, production-map activation, readiness, goal admission, named-location navigation, and controlled recovery integration                    |
| [`savo_realsense`](../../savo_ws/src/edge/savo_realsense/README.md)       |   0.1.0 | `edge/`       | Edge                                | Intel RealSense D435 launch/configuration, stream health, camera topic monitoring, and front-depth extraction                                               |
| [`savo_speech`](../../savo_ws/src/edge/savo_speech/README.md)             |   0.1.0 | `edge/`       | Edge, optional at startup           | ReSpeaker capture, wake/VAD/utterance handling, bounded SavoMind speech transport, returned-audio validation, playback, microphone gating, and speech state |
| [`savo_ui`](../../savo_ws/src/edge/savo_ui/README.md)                     |   0.1.0 | `edge/`       | Edge, optional at startup           | Read-only 800×480 framebuffer UI for robot state, safety, navigation, mapping, locations, speech, and power presentation                                    |
| [`savo_vo`](../../savo_ws/src/edge/savo_vo/README.md)                     |   0.1.0 | `edge/`       | Edge                                | RGB-D visual odometry production, VO health, diagnostics, and odometry republishing                                                                         |
| [`savo_bridge`](../../savo_ws/src/shared/savo_bridge/README.md)           |   0.1.0 | `shared/`     | Edge                                | Typed and bounded ROS 2 ↔ SavoMind observation/command boundary; fail-closed STOP, cancellation, teleoperation, navigation, mapping, and query adapters     |
| [`savo_bringup`](../../savo_ws/src/shared/savo_bringup/README.md)         |   0.6.0 | `shared/`     | Core and Edge                       | Distributed launch orchestration, role selection, mode/profile validation, and independent Core/Edge readiness aggregation                                  |
| [`savo_description`](../../savo_ws/src/shared/savo_description/README.md) |   0.1.0 | `shared/`     | Core and Edge; observer dependency  | URDF/Xacro, fixed robot TF, geometry profile, wheel/sensor frames, generated footprint, RViz assets, and geometry validation                                |
| [`savo_msgs`](../../savo_ws/src/shared/savo_msgs/README.md)               |   0.9.0 | `shared/`     | Core and Edge; interface dependency | Shared ROS 2 messages, services, and actions for status, navigation, AprilTags, locations, mapping missions, authority, and release contracts               |
| [`savo_observer`](../../savo_ws/src/shared/savo_observer/README.md)       |   0.1.0 | `shared/`     | Operator workstation                | Read-only telemetry, browser dashboard, RViz profiles, connection checks, and bounded observer views                                                        |
| [`savo_perception`](../../savo_ws/src/shared/savo_perception/README.md)   |   0.1.0 | `shared/`     | Core and Edge                       | Core: ToF/ultrasonic safety fusion and command gate. Edge: optional filtered D435 obstacle-cloud producer. C++ is the production path                       |
| [`savo_power`](../../savo_ws/src/shared/savo_power/README.md)             |   0.1.0 | `shared/`     | Core and Edge                       | Role-specific UPS monitoring, base-battery monitoring, aggregate power state, health, diagnostics, and shutdown requests                                    |
| [`savo_supervisor`](../../savo_ws/src/shared/savo_supervisor/README.md)   |   0.1.0 | `shared/`     | Core                                | Robot-wide readiness, mission authority, startup arming, fault latching, persistent system state, map-context authorization, and controlled shutdown        |

## Authority boundaries

### Motion execution

```text
Approved command source
        ↓
savo_control
        ↓
savo_perception command safety gate
        ↓
/cmd_vel_safe
        ↓
savo_base
        ↓
drivetrain hardware
```

Only `savo_base` executes drivetrain output. No Edge package, SavoMind component, UI, observer, mapper, or supervisor may write directly to the motor hardware.

### Permission versus execution

`savo_supervisor` owns permission and system-state authority. It does not:

* command motors;
* select frontiers;
* plan or execute Nav2 paths;
* save maps;
* approve semantic locations on behalf of an operator;
* bypass package-owned readiness checks.

The package that owns an operation remains responsible for executing it and for revalidating supervisor authority during operation.

### Edge command boundary

`savo_bridge` is deployed on Edge because SavoMind uses local Unix-domain sockets there. The bridge may expose only explicitly typed operations. It must fail closed on stale state, invalid map context, missing authority, malformed input, unavailable services/actions, or timeout.

The bridge is not allowed to provide:

* arbitrary topic publication;
* generic service or action forwarding;
* shell execution;
* direct motor access;
* operator approval of map or location releases;
* supervisor or navigation-readiness bypasses.

### Presentation surfaces

`savo_ui` and `savo_observer` are read-only. They may subscribe, aggregate, format, and display state, but they must not acquire motion or mission authority.

### Interface package

`savo_msgs` contains generated interface definitions only. It has no independent runtime node and does not own behavior.

## Shared-package deployment details

| Shared package     | Core | Edge |  Observer workstation  | Notes                              |
| ------------------ | :--: | :--: | :--------------------: | ---------------------------------- |
| `savo_msgs`        |  Yes |  Yes | Dependency as required | Generated interfaces               |
| `savo_description` |  Yes |  Yes |           Yes          | Shared physical and TF contract    |
| `savo_bringup`     |  Yes |  Yes |           No           | Role-selecting production launch   |
| `savo_perception`  |  Yes |  Yes |           No           | Different role-specific components |
| `savo_power`       |  Yes |  Yes |    Observed remotely   | Separate Core and Edge launches    |
| `savo_supervisor`  |  Yes |  No  |    Observed remotely   | Core authority only                |
| `savo_bridge`      |  No  |  Yes |           No           | Edge-local SavoMind boundary       |
| `savo_observer`    |  No  |  No  |           Yes          | Read-only operator tooling         |

## Production startup defaults

The distributed entry point is:

```bash
ros2 launch savo_bringup robot_bringup.launch.py host_role:=core
ros2 launch savo_bringup robot_bringup.launch.py host_role:=edge
```

The deployment wrappers preserve these defaults:

* `robot_mode=safe_idle`;
* `bringup_profile=lidar_only`;
* Core control mode `STOP`;
* provisional geometry not allowed for motion;
* D435 VoxelLayer validation `false`;
* Edge bridge enabled;
* Edge speech and UI disabled until explicitly enabled;
* optional Edge obstacle cloud disabled until hardware validation.

## Retired or non-existent package names

The following names are not packages in the current workspace and must not be added to role arrays or production launch files:

| Name             | Current disposition                                                                                                                                      |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `savo_intent`    | No ROS package. Intent/LLM reasoning belongs to SavoMind; approved robot actions cross through `savo_bridge`                                             |
| `savo_dashboard` | No ROS package. Operator telemetry belongs to `savo_observer`; robot display belongs to `savo_ui`; package-specific diagnostics remain with their owners |
| `savo_uwb`       | No current ROS package manifest. A staging directory exists under `savo_ws/src/future/`; promotion requires an explicit package and ownership review       |
| `future/`        | Non-package staging content exists but contains no `package.xml`; it is excluded from the 20-package workspace and production role arrays                  |

The obsolete central page `docs/packages/savo_intent.md` was removed during the Phase 1 documentation correction.

All 20 current packages now have source-reconciled central pages in this directory. Package-local README links in the matrix remain implementation references; the [documentation index](../README.md) links the central integration pages.

## Change-control rules

A package-role change must update and validate all of the following in the same change:

1. `deploy/core/env_core.sh` or `deploy/edge/env_edge.sh`;
2. package dependencies and installation rules;
3. role build and dependency-install tests;
4. distributed bringup requirements;
5. systemd/runtime preparation when applicable;
6. this ownership matrix;
7. the current system status and affected package documentation.

Do not maintain independent package lists in additional scripts. Deployment commands must consume the authoritative role arrays.
