# Package Ownership Matrix

Robot Savo uses one ROS 2 repository for both Raspberry Pi units, but packages are grouped by responsibility.

The source layout is role-based:

```text
Savo_Pi/savo_ws/src/
├── core/
├── edge/
├── shared/
└── future/
```

`core/` packages mainly run on `savo-core`.
`edge/` packages mainly run on `savo-edge`.
`shared/` packages are used by both Pis.
`future/` packages are planned or optional.

## Ownership summary

| Package             | Folder    | Primary Pi  | Role                                                                             |
| ------------------- | --------- | ----------- | -------------------------------------------------------------------------------- |
| `savo_base`         | `core/`   | `savo-core` | Low-level motor execution, mecanum drive output, base watchdog, base diagnostics |
| `savo_control`      | `core/`   | `savo-core` | Command shaping, twist mux, mode control, rotate/heading/recovery behavior       |
| `savo_localization` | `core/`   | `savo-core` | Wheel odometry, IMU integration, EKF fusion, localization diagnostics            |
| `savo_mapping`      | `core/`   | `savo-core` | Manual mapping, SLAM workflows, autonomous mapping/exploration support           |
| `savo_nav`          | `core/`   | `savo-core` | Nav2 configuration, AMCL/localization setup, navigation bringup                  |
| `savo_locations`    | `core/`   | `savo-core` | Campus destinations, known locations, semantic map data                          |
| `savo_vo`           | `edge/`   | `savo-edge` | Visual odometry from camera/RealSense input                                      |
| `savo_speech`       | `edge/`   | `savo-edge` | Speech input/output, STT/TTS bridge, wake/TTS gates                              |
| `savo_ui`           | `edge/`   | `savo-edge` | Display UI, robot face, navigation camera/status view                            |
| `savo_intent`       | `edge/`   | `savo-edge` | Intent bridge between user text/speech and robot action requests                 |
| `savo_msgs`         | `shared/` | both        | Shared custom ROS 2 messages                                                     |
| `savo_description`  | `shared/` | both        | URDF/Xacro, fixed TF frames, RViz model, sensor frame layout                     |
| `savo_perception`   | `shared/` | both        | Range/depth perception, safety stop, slowdown, cmd_vel safety gate               |
| `savo_bringup`      | `shared/` | both        | Role-based launch profiles for `savo-core` and `savo-edge`                       |
| `savo_dashboard`    | `shared/` | both        | Diagnostics dashboards and monitoring tools                                      |
| `savo_head`         | `future/` | optional    | Pan-tilt/head control if separated later                                         |
| `savo_uwb`          | `future/` | optional    | Future UWB localization support                                                  |

## `savo-core` package set

`savo-core` owns movement authority and robot-side autonomy.

Typical packages built on `savo-core`:

```text
savo_msgs
savo_description
savo_perception
savo_base
savo_control
savo_localization
savo_mapping
savo_nav
savo_locations
savo_dashboard
savo_bringup
```

`savo-core` should run the packages responsible for:

* motor output
* command control
* safety gating
* wheel odometry
* IMU/EKF localization
* mapping
* Nav2 navigation
* campus location lookup
* core diagnostics

## `savo-edge` package set

`savo-edge` owns heavier helper tasks.

Typical ROS 2 packages built on `savo-edge`:

```text
savo_msgs
savo_description
savo_perception
savo_vo
savo_speech
savo_ui
savo_intent
savo_dashboard
savo_bringup
```

`savo-edge` should run the packages responsible for:

* RealSense/depth processing
* visual odometry
* speech input/output
* display UI
* intent/helper bridge
* edge diagnostics

`savo-edge` also runs Docker services from the separate `Robot_Savo_Server` repository.

Typical server services:

```text
robot-llm
robot-stt
```

`robot-stt` may remain optional if speech is handled directly in the ROS 2 speech stack.

## Shared package rules

Shared packages are not automatically run everywhere.

A shared package means:

* both Pis may need the package installed
* different nodes from the same package may run on different Pis
* the package contains common assets, messages, config, or launch logic

Example:

| Package            | On `savo-core`                                     | On `savo-edge`                               |
| ------------------ | -------------------------------------------------- | -------------------------------------------- |
| `savo_perception`  | ToF, ultrasonic, safety stop, `/cmd_vel_safe` gate | RealSense/depth front-min, depth/3D helpers  |
| `savo_description` | TF for base, wheels, LiDAR, IMU, Nav2              | TF for RealSense, VO, UI camera/depth frames |
| `savo_msgs`        | Custom message definitions                         | Custom message definitions                   |
| `savo_bringup`     | `savo_core.launch.py`                              | `savo_edge.launch.py`                        |

## Motion authority rule

Only `savo-core` owns final robot motion.

Packages on `savo-edge` can support perception, interaction, UI, VO, speech, and AI helper tasks, but they do not directly command the motor board.

Motion must pass through:

```text
request source
        ↓
savo_control
        ↓
savo_perception safety gate
        ↓
/cmd_vel_safe
        ↓
savo_base
        ↓
motors
```

## Build policy

Robot Savo should not build every package on every Pi by default.

Build only the packages needed for the role:

| Pi          | Build style                                                                                    |
| ----------- | ---------------------------------------------------------------------------------------------- |
| `savo-core` | core + shared packages needed for movement, safety, localization, mapping, and navigation      |
| `savo-edge` | edge + shared packages needed for RealSense/depth, VO, speech, UI, intent, and helper services |

This keeps dependencies smaller and makes debugging easier.

## Future packages

Packages under `future/` are not part of the active production path until they are needed.

| Package     | Status                                                     |
| ----------- | ---------------------------------------------------------- |
| `savo_head` | Keep only if pan-tilt/head control becomes its own package |
| `savo_uwb`  | Keep for later UWB localization experiments                |

Heading control is not a separate package. Heading, rotate, and yaw-hold behavior belong inside `savo_control`.
