# Component Validation Overview

## Purpose

Robot Savo validates each component from source/static checks through non-actuating detection and only then controlled hardware integration. Direct hardware tools are permitted only where the owning test plan identifies them, the component is physically isolated, and the tool cannot bypass a production safety boundary.

Test existence is not PASS. Use [test evidence guidelines](test_evidence_guidelines.md), the [result template](test_result_template.md), and [failure/abort criteria](failure_and_abort_criteria.md).

## Required order

1. Record commit, host, hardware/config/geometry identity and test IDs.
2. Run package source/unit/PC checks.
3. Inspect wiring/permissions and detect the device with actuators de-energized.
4. Verify non-actuating ROS data, frames, timestamps, freshness and fault state.
5. Pass the owning subsystem prerequisites.
6. Perform controlled hardware tests at minimum safe ranges.
7. Integrate producers/consumers and test disconnect/restart/recovery.
8. Review evidence before allowing a dependent stage.

Motor and servo discovery does not authorize raw output. Drivetrain testing uses `/cmd_vel_manual -> savo_control -> savo_perception -> /cmd_vel_safe -> savo_base`; head motion uses its bounded controller. Keep control `STOP` during sensor tests.

## Device and owner map

| Component | Owner | Host | Detailed plan | Phase 7 status |
| --- | --- | --- | --- | --- |
| PCA9685/motors | `savo_base` (chip initialization interaction with head requires review) | Core | [Base](base_test_plan.md) | Current hardware regression required |
| Four encoders/BNO055 | `savo_localization` | Core | [Localization](localization_test_plan.md) | Current hardware regression required |
| RPLIDAR A1 | `savo_lidar` | Core | [LiDAR](lidar_test_plan.md) | Current hardware regression required |
| ToFs/TCA9548A/ultrasonic | `savo_perception` | Core | [Perception](perception_test_plan.md) | Threshold/identity regression required |
| Pan/tilt/Pi Camera | `savo_head` | Core | [Head](head_test_plan.md) | Limits/mount/camera regression required |
| Core/Edge UPS and base ADC | `savo_power` | Both | [Power](power_test_plan.md) | Calibration/fault testing required |
| RealSense D435 | `savo_realsense` | Edge | [RealSense](realsense_test_plan.md) | USB/profile/restart required |
| ReSpeaker/speaker | `savo_speech` | Edge | [Speech](speech_test_plan.md) | ALSA/acoustic validation required |
| Display/touch | `savo_ui` | Edge | [UI](ui_test_plan.md) | Framebuffer/touch validation required |

## Evidence

Retain terminal logs, device identity, configuration, frame/rate/timestamp samples, measurements/instrument identity, fault/restart results and physical media where useful. Small reviewed artifacts may live under `docs/assets/testing`; large/sensitive bags/audio/video belong in controlled external storage with a digest manifest.

The older [Core](savo_core_component_validation.md) and [Edge](savo_edge_component_validation.md) sheets are status summaries only. The detailed plans are authoritative.
