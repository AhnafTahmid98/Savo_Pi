# Robot Savo

Robot Savo is an indoor campus guide robot built with ROS 2, Raspberry Pi, LiDAR, depth sensing, wheel odometry, speech interaction, and AI-assisted intent handling.

The robot is designed to map indoor environments, localize itself, navigate to named campus locations, interact with people through voice, and operate safely around humans.

This repository contains the main ROS 2 workspace, hardware validation tools, diagnostics, launch files, configuration files, and project documentation for the Robot Savo platform.

---

## System Architecture

Robot Savo uses a two-Pi architecture.

| Device | Hostname | Role |

|---|---|---|
| Main robot controller | `savo-core` | Movement authority, base control, safety, LiDAR, localization, Nav2 |
| Edge/helper computer | `savo-edge` | RealSense, speech, display, visual odometry, AI helper services |

The main controller keeps all movement-critical responsibility. The edge computer handles heavier perception and interaction workloads.

---

## Main Capabilities

- Indoor 2D mapping with LiDAR
- Autonomous navigation with ROS 2 Nav2
- Mecanum-wheel base control
- Safety-gated velocity control
- Wheel encoder and IMU based localization
- RealSense depth sensing
- Visual odometry support from the edge computer
- Speech input and text-to-speech output
- AI/LLM intent handling
- Display UI for interaction, navigation, and mapping modes

Supported high-level robot intents include:

| Intent | Meaning |

|---|---|
| `NAVIGATE` | Guide the user to a known location |
| `FOLLOW` | Follow a person |
| `STOP` | Stop robot movement |
| `STATUS` | Report current robot state |
| `CHATBOT` | Social or informational conversation |

---

## Repository Layout

```text
Savo_Pi/
├── docs/                  # Project documentation
├── savo_ws/               # ROS 2 Jazzy workspace
├── tools/                 # Hardware checks, diagnostics, and helper scripts
├── .gitignore
├── LICENSE
└── README.md
```

The ROS 2 source code is stored in:

```text
savo_ws/src/
```

Project documentation is stored in:

```text
docs/
```

Hardware validation scripts and standalone diagnostic tools are stored in:

```text
tools/
```

---

## Hardware Overview

| Area | Hardware |

|---|---|
| Compute | Raspberry Pi 5 units for `savo-core` and `savo-edge` |
| Base | Freenove 4WD mecanum platform |
| Motor control | PCA9685-based motor control |
| Odometry | 4 wheel encoders |
| IMU | BNO055 |
| LiDAR | RPLIDAR A1 |
| Depth camera | Intel RealSense D435 |
| Near-field safety | VL53L1X ToF sensors and ultrasonic sensor |
| Audio | ReSpeaker microphone array and speakers |
| Display | 7-inch DSI touchscreen |
| Power | UPS HAT / battery-backed Raspberry Pi power system |

---

## Physical Layer Layout

Robot Savo is organized into four physical layers.

| Layer | Main Components |

|---|---|
| Layer 0 / Base | Freenove board, motors, mecanum wheels, ToF sensors, ultrasonic sensor |
| Layer 1 | `savo-core`, breadboard, speakers, display |
| Layer 2 | ReSpeaker mic array, `savo-edge`, RealSense camera, pan-tilt camera |
| Layer 3 / Top | RPLIDAR A1 |

---

## Software Stack

| Layer | Technology |

|---|---|
| Operating system | Ubuntu Server 24.04 |
| Robotics middleware | ROS 2 Jazzy |
| Mapping | SLAM Toolbox |
| Navigation | Nav2 |
| Localization | robot_localization EKF |
| Visualization | RViz on external PC/laptop |
| Speech-to-text | Faster-Whisper |
| Text-to-speech | Piper |
| AI server | FastAPI-based Robot Savo Server |
| Deployment | ROS 2 launch files and Docker services |

---

## Main ROS 2 Packages

Robot Savo is organized into focused ROS 2 packages. Each package owns one clear part of the robot.

| Package | Responsibility |

|---|---|
| `savo_base` | Base driver, mecanum motor control, watchdogs, base diagnostics |
| `savo_control` | Velocity shaping, control modes, twist muxing, recovery logic |
| `savo_lidar` | RPLIDAR ownership, filtering, health checks, scan publishing |
| `savo_perception` | Range sensors, depth front-min processing, safety stop logic |
| `savo_localization` | Wheel odometry, IMU input, EKF localization |
| `savo_vo` | Visual odometry support from RealSense |
| `savo_realsense` | RealSense camera ownership and validation |
| `savo_speech` | STT, TTS, wake-word flow, speech bridge |
| `savo_intent` | Robot intent client and LLM server bridge |
| `savo_nav` | Nav2 integration, navigation parameters, map usage |
| `savo_mapping` | Mapping workflow and SLAM integration |
| `savo_description` | Robot URDF, frames, physical model, transforms |

---

## Safety Model

Robot Savo does not send raw navigation commands directly to the motors.

The command path is:

```text
/cmd_vel
   ↓
safety and slowdown layer
   ↓
/cmd_vel_safe
   ↓
base driver
   ↓
motor controller
```

Safety inputs include:

- LiDAR obstacle awareness
- RealSense front depth distance
- ToF range sensors
- Ultrasonic backup sensing
- Emergency stop state
- Command timeout watchdogs
- Stale sensor checks

The base driver only consumes `/cmd_vel_safe`.

---

## Two-Pi Deployment

### `savo-core`

`savo-core` runs the movement-critical stack:

- base driver
- safety gate
- LiDAR
- wheel odometry
- IMU
- EKF localization
- Nav2
- mapping stack

### `savo-edge`

`savo-edge` runs helper and heavier workloads:

- RealSense camera
- visual odometry
- ReSpeaker microphone
- STT/TTS
- display UI
- Robot Savo Server Docker services

Movement authority stays on `savo-core`.

---

## Robot Savo Server

The AI helper server is kept as a separate project from this ROS 2 repository.

Expected services on `savo-edge`:

| Service | Port | Purpose |

|---|---:|---|
| `robot-llm` | `8000` | Intent handling, chatbot replies, navigation command interpretation |
| `robot-stt` | `9000` | Speech-to-text service |

The ROS 2 robot stack communicates with the server over the local robot network.

---

## Build Instructions

From the ROS 2 workspace:

```bash
cd ~/Savo_Pi/savo_ws

source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

Build one package:

```bash
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

Example:

```bash
colcon build --symlink-install --packages-select savo_lidar
source install/setup.bash
```

---

## Basic Development Workflow

Typical workflow on a Raspberry Pi:

```bash
cd ~/Savo_Pi
git pull

cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Basic ROS 2 checks:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan
```

---

## Hardware Validation Workflow

Hardware-facing packages should be validated before full robot bring-up.

Recommended order:

1. Power system
2. I2C devices
3. USB devices
4. LiDAR
5. RealSense
6. Audio devices
7. IMU
8. Encoders
9. Motor direction
10. Safety stop
11. `/cmd_vel_safe`
12. Full base bring-up
13. Localization
14. Mapping
15. Navigation

Do not run autonomous navigation until base control, localization, and safety behavior are verified.

---

## Operating Notes

This repository controls real robot hardware.

Before running motor or navigation commands:

1. Lift the robot wheels off the ground during motor direction tests.
2. Confirm `/safety/stop` is working.
3. Confirm `/cmd_vel_safe` is published correctly.
4. Confirm emergency stop behavior.
5. Keep the robot in a clear test area.
6. Start with low speed limits.
7. Do not run Nav2 until localization and safety are stable.

---

## Documentation

Project documentation is stored in:

```text
docs/
```

Recommended documentation areas:

```text
docs/architecture/
docs/hardware/
docs/packages/
docs/testing/
docs/assets/
```

Photos, screenshots, wiring images, and validation proof can be stored under:

```text
docs/assets/
```

Demo videos can be linked from documentation after they are uploaded.

---

## Current Development Status

Robot Savo is under active hardware and ROS 2 integration.

Recently validated areas include:

- RPLIDAR A1 hardware detection and non-ROS diagnostic testing
- DSI display detection on `savo-edge`
- RealSense USB detection on `savo-edge`
- ReSpeaker microphone detection on `savo-edge`
- ROS 2 package ownership work for LiDAR, base, perception, control, localization, and RealSense
- two-Pi architecture planning and setup

The next development steps focus on real hardware bring-up, package-level validation, and full robot assembly testing.

---

## Git Workflow

After editing files:

```bash
git status
git add .
git commit -m "docs: update project documentation"
git push
```

For README-only changes:

```bash
git add README.md
git commit -m "docs: add main project README"
git push
```

---

## License

This project is proprietary unless stated otherwise.

---

## Maintainer

Ahnaf Tahmid  
Robot Savo Project
