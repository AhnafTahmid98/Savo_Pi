# Component Validation Overview

Robot Savo validates each physical component in isolation before integrating it into a ROS 2 package. This catches wiring faults, wrong I2C addresses, driver incompatibilities, and misconfigured peripherals before they become harder-to-debug ROS issues.

## Process

### 1. Test hardware alone first

Run a standalone script or CLI tool that talks directly to the hardware over the relevant interface (I2C, GPIO, USB, audio, serial). No ROS, no launch files.

The goal is to confirm the physical component works and the interface is wired correctly.

Examples:

| Component          | Standalone test                          |
| ------------------ | ---------------------------------------- |
| BNO055 IMU         | `tools/diag/sensors/imu_test.py`         |
| Wheel encoders     | `tools/diag/motion/encoders_test.py`     |
| RPLIDAR A1         | `rplidar_ros` driver, `ros2 topic echo`  |
| UPS HAT            | `bat.py`, `i2cdetect -y 1`               |
| RealSense D435     | `realsense-viewer` or `rs-enumerate-devices` |
| ReSpeaker mic      | `arecord -l`, `aplay` passthrough        |

### 2. Confirm the interface works

Check that the host can see the device:

- I2C: `sudo i2cdetect -y 1` shows the correct address
- USB: `lsusb` shows the device
- GPIO: pins claimed without error, signal readable
- Audio: device appears in `arecord -l` / `aplay -l`

### 3. Save command output or screenshot

Record proof before moving on. Store in `docs/assets/`:

```
docs/assets/hardware/        ← photos, wiring, HAT photos
docs/assets/testing/         ← terminal output screenshots, RViz captures
docs/assets/screenshots/     ← UI or tool screenshots
```

Paste key output lines into the relevant validation table (see below).

### 4. Integrate into the ROS 2 package

Only after standalone validation passes, wire the component into the relevant ROS 2 package:

- Add the driver node or hardware interface
- Write or update the launch file
- Publish to the correct topic

### 5. Test package-level behaviour

Confirm the ROS 2 package publishes correct data:

```bash
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

Check message content, frame IDs, and publish rate against the expected values in the package test plan.

## Validation tables

Component-level validation status lives in two separate files:

| File | Covers |
| ---- | ------ |
| [`savo_core_component_validation.md`](savo_core_component_validation.md) | `savo-core` hardware: motors, encoders, IMU, LiDAR, ToF, UPS |
| [`savo_edge_component_validation.md`](savo_edge_component_validation.md) | `savo-edge` hardware: RealSense, mic, speaker, STT, TTS, VO, Docker |

Package-level test plans live in the same `testing/` folder, named per package.
