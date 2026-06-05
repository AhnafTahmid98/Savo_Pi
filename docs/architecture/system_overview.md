# Robot Savo System Overview

Robot Savo is an AI-assisted autonomous indoor guide robot designed for campus navigation, voice interaction, mapping, and safe human-friendly movement.

The robot is built as a real ROS 2 system, not only a demo prototype. The main goal is to guide people indoors, understand simple user commands, explain its current state, avoid obstacles, and operate safely around humans.

## System goal

Robot Savo should be able to:

* map indoor campus areas
* localize itself on the map
* navigate to known campus locations
* respond to voice commands
* show interaction/navigation status on its display
* detect obstacles using multiple sensors
* stop or slow down before unsafe motion
* separate AI interaction from motor authority

The robot is designed around real hardware constraints: Raspberry Pi compute limits, sensor timing, ROS 2 networking, motor safety, physical mounting, power stability, and indoor navigation reliability.

## Compute architecture

Robot Savo uses two Raspberry Pi units:

| Unit        | Role                                                                                   |
| ----------- | -------------------------------------------------------------------------------------- |

| `savo-core` | Movement authority, motor execution, safety decisions, localization, mapping, and Nav2 |
| `savo-edge` | RealSense/depth processing, visual odometry, speech/audio, UI, and Dockerized AI/helper services from `Robot_Savo_Server`|

Both units use the same `Savo_Pi` ROS 2 repository. They do not build or run every package. Each Pi builds only the packages needed for its role.

## Repository split

Robot Savo uses two repositories:

| Repository | Runs on | Responsibility |
| --- | --- | --- |
| `Savo_Pi` | `savo-core` and `savo-edge` | ROS 2 robot packages, launch profiles, hardware interfaces, perception, localization, mapping, navigation, speech/UI bridges, and shared robot description |
| `Robot_Savo_Server` | `savo-edge` | Dockerized AI/helper services such as LLM, intent support, and optional STT-related services |

The server repo is kept separate because it has different dependencies, Docker services, runtime files, API keys, and model-related assets.

The AI/helper server can support interaction and intent handling, but movement authority remains on `savo-core`.

## ROS 2 workspace layout

The workspace is organized by package role:

```text
Savo_Pi/savo_ws/src/
├── core/
├── edge/
├── shared/
└── future/
```

| Folder    | Meaning                             |
| --------- | ----------------------------------- |
| `core/`   | Packages mainly used by `savo-core` |
| `edge/`   | Packages mainly used by `savo-edge` |
| `shared/` | Packages needed by both Pis         |
| `future/` | Optional or later packages          |

This layout keeps the repo easy to understand for developers while still allowing one shared codebase.

## Main subsystems

Robot Savo is divided into the following system areas.

| Subsystem             | Main packages       | Responsibility                                                              |
| --------------------- | ------------------- | --------------------------------------------------------------------------- |
| Base execution        | `savo_base`         | Motor control, mecanum drive output, watchdog, base hardware diagnostics    |
| Control               | `savo_control`      | Command shaping, twist mux, control modes, rotate/heading/recovery behavior |
| Perception and safety | `savo_perception`   | Range sensors, depth front-min, safety stop, slowdown, `/cmd_vel_safe` gate |
| Localization          | `savo_localization` | Wheel odometry, IMU, EKF fusion, later VO input                             |
| Description           | `savo_description`  | URDF/Xacro, TF frames, RViz model, sensor frame layout                      |
| Mapping               | `savo_mapping`      | Manual mapping and autonomous mapping workflows                             |
| Navigation            | `savo_nav`          | Nav2, AMCL/localization configs, planners, controllers, costmaps            |
| Visual odometry       | `savo_vo`           | VO from edge-side camera/RealSense pipeline                                 |
| Speech                | `savo_speech`       | STT, TTS, speech bridge, audio interaction                                  |
| Intent                | `savo_intent`       | LLM/intent bridge between speech/user text and robot actions                |
| UI                    | `savo_ui`           | Display face, camera/navigation view, interaction UI                        |
| Bringup               | `savo_bringup`      | Core and edge launch profiles                                               |
| Messages              | `savo_msgs`         | Shared custom ROS 2 messages                                                |

## Motion authority

Only `savo-core` owns movement authority.

High-level systems such as voice, AI, UI, or edge perception may request actions, but they do not directly control the motors. Motion commands must pass through the control and safety layers before reaching the base driver.

The expected motion path is:

```text
Nav2 / teleop / intent request
        ↓
savo_control
        ↓
cmd_vel command
        ↓
savo_perception safety gate
        ↓
/cmd_vel_safe
        ↓
savo_base
        ↓
motor board and wheels
```

This keeps the robot safer because AI and user interaction are separated from final motor execution.

## Safety model

Robot Savo uses layered safety.

| Layer             | Role                                                    |
| ----------------- | ------------------------------------------------------- |
| Command control   | Limits and shapes velocity commands                     |
| Perception safety | Detects nearby obstacles using range/depth sensors      |
| Safety gate       | Blocks or scales unsafe velocity commands               |
| Base watchdog     | Stops motors if commands become stale                   |
| Manual testing    | Verifies behavior on real hardware before full autonomy |

The safety system is designed so that the robot can stop even if a higher-level behavior fails.

## Perception and obstacle detection

Robot Savo uses multiple sensors because one sensor type is not enough for reliable indoor navigation.

| Sensor          | Role                                                      |
| --------------- | --------------------------------------------------------- |
| 2D LiDAR        | SLAM, planar obstacle detection, Nav2 obstacle layer      |
| RealSense/depth | 3D obstacle awareness and depth-based front detection     |
| ToF sensors     | Close-range side/front safety support                     |
| Ultrasonic      | Close-range backup safety sensor                          |
| Camera          | Visual feedback, pan-tilt camera use, future visual tasks |
| IMU             | Orientation and localization support                      |
| Wheel encoders  | Wheel odometry and motion feedback                        |

The navigation stack will use 2D LiDAR as the main mapping/navigation backbone and add depth/3D costmap data for better obstacle detection during navigation.

## Localization direction

Robot Savo localization is planned around:

* four wheel encoders
* IMU
* visual odometry from `savo-edge`
* EKF fusion

The robot should use wheel odometry and IMU as core localization inputs. VO can later improve motion estimation, especially when wheel odometry is affected by mecanum slip.

## Mapping and navigation direction

Mapping and navigation are built around ROS 2 tools such as SLAM, Nav2, costmaps, localization, and RViz2 debugging.

The expected navigation stack uses:

* 2D LiDAR for SLAM and main obstacle detection
* Nav2 for path planning and control
* local and global costmaps
* 3D/depth obstacle input where useful
* safety gating before motor output

`/cmd_vel_safe` remains the final command topic before `savo_base`.

## Robot description and TF

`savo_description` is the shared physical truth package.

It defines:

* robot body frames
* wheel frames
* LiDAR frame
* IMU frame
* RealSense/depth camera frames
* ToF and ultrasonic frames
* display and ReSpeaker frames
* `savo-core` and `savo-edge` mounting frames
* RViz model views

The standard TF chain is:

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── wheels
            ├── sensors
            ├── camera/depth frames
            ├── display/audio frames
            └── compute unit frames
```

`map -> odom` and `odom -> base_footprint` are produced by localization/navigation systems. Fixed robot frames are published through the robot description.

## Physical layout

Robot Savo is currently planned as a four-layer robot.

| Layer          | Main contents                                                          |
| -------------- | ---------------------------------------------------------------------- |
| Layer 0 / base | Freenove board, motors, wheels, ToF, ultrasonic                        |
| Layer 1        | `savo-core` Pi, breadboard, speakers, display                          |
| Layer 2        | ReSpeaker mic array, `savo-edge` Pi, RealSense camera, pan-tilt camera |
| Layer 3 / top  | LiDAR only                                                             |

Final URDF dimensions should be updated after the printed parts are installed and measured.

## AI boundary

Robot Savo can use AI for interaction, intent understanding, status replies, and helper reasoning. The Dockerized AI/helper server runs on `savo-edge` from the separate `Robot_Savo_Server` repo.

AI services do not directly drive the motors. AI output must pass through ROS 2 integration, intent handling, navigation/control logic, and the safety chain before any motor command reaches `savo_base`.

Movement authority remains on `savo-core`.

## Current engineering status

The project is being developed package by package. The current approach is:

1. keep ROS 2 robot code in the `Savo_Pi` repository
2. keep Dockerized AI/helper services in the separate `Robot_Savo_Server` repo
3. organize ROS 2 packages by role
4. build packages only where needed
5. validate each layer separately
6. collect screenshots, logs, and test proof
7. update docs from real measurements and real test results

This document is the top-level overview. More detailed design is documented in the architecture, hardware, package, deployment, and testing folders.
