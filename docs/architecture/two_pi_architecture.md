# Two-Pi Architecture

Robot Savo uses two Raspberry Pi units: `savo-core` and `savo-edge`.

The split is based on responsibility, not just performance. `savo-core` owns movement authority and real-time robot safety. `savo-edge` handles heavier helper work such as RealSense/depth processing, visual odometry, speech/audio, UI, and Dockerized AI helper services.

## Summary

| Unit        | Main role                                                                                 |
| ----------- | ----------------------------------------------------------------------------------------- |
| `savo-core` | Movement authority, base control, safety decisions, localization, mapping, and navigation |
| `savo-edge` | RealSense/depth, visual odometry, speech/audio, UI, and AI/helper services                |

Both Pis use the same `Savo_Pi` GitHub repository for ROS 2 packages. `savo-edge` also runs the separate `Robot_Savo_Server` repository as Docker services.

## Repository split

Robot Savo uses two main repositories.

| Repository          | Runs on                     | Role                                                                                                                                                                  |
| ------------------- | --------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Savo_Pi`           | `savo-core` and `savo-edge` | ROS 2 workspace for robot packages, launch files, hardware interfaces, perception, localization, mapping, navigation, speech/UI bridges, and shared description files |
| `Robot_Savo_Server` | `savo-edge`                 | Dockerized AI/helper server for LLM, intent/helper services, and optional STT-related services                                                                        |

`Robot_Savo_Server` is separate because it has different dependencies, Docker services, runtime state, model assets, API keys, and server-specific deployment needs.

The AI/helper server supports interaction and reasoning, but it does not directly control motors.

## `savo-core`

`savo-core` is the main robot authority computer.

It owns:

* motor command execution
* mecanum base control
* command shaping and mode control
* safety stop and slowdown decisions
* `/cmd_vel_safe` gating
* wheel odometry
* IMU integration
* EKF/localization
* mapping
* Nav2 navigation
* production robot startup for motion-related systems

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

`savo-core` is the only Pi that should be able to send final motor commands to the base driver.

## `savo-edge`

`savo-edge` is the helper and heavier-processing computer.

It handles:

* RealSense/depth camera processing
* depth front-min or depth obstacle helper data
* visual odometry
* speech/audio processing
* TTS/STT bridge tasks
* robot display UI
* LLM/intent helper communication
* Dockerized `Robot_Savo_Server` services

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

Typical Docker services from `Robot_Savo_Server`:

```text
robot-llm
robot-stt
```

`robot-stt` may be optional if speech is handled directly inside the ROS 2 speech stack.

## Shared packages

Some ROS 2 packages are shared because both Pis need the same message types, frame model, launch profiles, or diagnostics.

| Package            | Why it is shared                                                            |
| ------------------ | --------------------------------------------------------------------------- |
| `savo_msgs`        | Custom message definitions used across the robot                            |
| `savo_description` | Common URDF/Xacro, TF frame model, RViz views, and sensor frame definitions |
| `savo_perception`  | Core uses safety/range nodes; edge may run RealSense/depth nodes            |
| `savo_bringup`     | Holds launch profiles for both Pi roles                                     |
| `savo_dashboard`   | Useful for diagnostics on either Pi                                         |

Shared does not mean every node runs on both Pis. It means the package contains code or assets used by both sides.

## Network connection

The two Pis will be connected directly over Ethernet.

Expected communication:

```text
savo-core  <---- Ethernet / ROS 2 / HTTP ---->  savo-edge
```

`savo-core` and `savo-edge` should use stable hostnames or fixed IP addresses. ROS 2 communication should be configured with the same `ROS_DOMAIN_ID`.

The server APIs on `savo-edge` should use stable addresses so ROS nodes and helper clients can reach them consistently.

Example service direction:

```text
savo-core ROS 2 nodes
        ↓
HTTP / ROS 2 network
        ↓
savo-edge helper services
```

Example AI/helper direction:

```text
user speech or text
        ↓
savo_speech / savo_intent
        ↓
Robot_Savo_Server on savo-edge
        ↓
intent result
        ↓
savo-core navigation/control request
```

## Motion authority boundary

`savo-edge` can support the robot, but it does not own motion authority.

The safe motion path is:

```text
intent / teleop / Nav2 request
        ↓
savo_control
        ↓
/cmd_vel
        ↓
savo_perception safety gate
        ↓
/cmd_vel_safe
        ↓
savo_base
        ↓
motor board
```

AI output, UI actions, speech commands, and edge-side perception must pass through the robot control and safety layers before motion reaches the motors.

This boundary is important because `savo-edge` may run heavier, less deterministic workloads such as AI services, camera processing, Docker containers, and UI tasks.

## Perception split

Perception is shared across both Pis.

| Perception function                    | Preferred Pi |
| -------------------------------------- | ------------ |
| ToF sensors                            | `savo-core`  |
| Ultrasonic sensor                      | `savo-core`  |
| Safety stop fusion                     | `savo-core`  |
| `/cmd_vel_safe` gate                   | `savo-core`  |
| RealSense depth processing             | `savo-edge`  |
| Depth point cloud / 3D obstacle helper | `savo-edge`  |
| Visual odometry                        | `savo-edge`  |

Depth and VO data from `savo-edge` can be sent to `savo-core` for localization, costmap use, or navigation support.

## Localization and navigation split

`savo-core` owns localization and navigation.

Main inputs:

* wheel odometry from four encoders
* IMU data
* VO from `savo-edge`
* LiDAR scan
* depth/3D obstacle support from `savo-edge`

Navigation should use 2D LiDAR as the main mapping/navigation backbone. Depth and 3D costmap layers can improve obstacle detection during navigation.

## Build strategy

The repository is shared, but builds are role-based.

`savo-core` should build the packages it needs for movement, safety, localization, mapping, and navigation.

`savo-edge` should build the packages it needs for RealSense/depth, VO, speech, UI, intent, and shared assets.

This avoids unnecessary dependencies on each Pi and keeps debugging cleaner.

## Startup direction

Production startup should eventually use separate services or launch profiles:

```text
savo-core.service
savo-edge.service
robot-savo-server.service or Docker Compose on savo-edge
```

The exact service names may change, but the role boundary should stay the same:

* `savo-core` starts robot movement, safety, localization, mapping, and navigation systems.
* `savo-edge` starts camera/depth, VO, speech, UI, and AI/helper services.

## Design rule

`savo-core` is allowed to move the robot.

`savo-edge` is allowed to help the robot understand, see, speak, display, and reason.

The robot should still be able to stop safely even if `savo-edge` or the AI/helper server fails.
