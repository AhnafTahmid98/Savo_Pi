# savo_head

## Purpose

Owns Core pan/tilt servos, Pi Camera, dynamic head TF, scan modes, AprilTag detection, and typed confirmation.

## Deployment

Core only. `head_bringup.launch.py` starts actuation/scan/TF/status; camera and AprilTag launches are composed when required.

## Responsibilities

Clamp/drive servos; publish joint state and calibrated dynamic TF; run staged scans; acquire/stream camera images; produce and validate AprilTag evidence.

## Non-responsibilities and authority boundaries

Does not persist/approve semantic locations, authorize mapping/navigation, or command the base. Tag detections are evidence only.

## Package structure

Production C++ nodes/drivers; `_py` scripts are fallbacks/diagnostics. Configuration owns servo, scan, frame, camera, and tag policy.

## Runtime components

### Production nodes

`head_controller_node`, `head_scan_node`, `head_tf_node`, `head_status_node`, `head_camera_status_node`, `apriltag_detector_node`, `apriltag_confirmation_action_node`, and compatibility `apriltag_confirm_node`. External `gscam_node` uses `libcamerasrc` for ROS images.

## Runtime data flow

`scan/manual -> controller -> servos -> JointState -> head TF`; `Pi Camera -> gscam -> detector -> ConfirmAprilTag -> mapping/nav`.

## ROS interfaces

### Published topics

`/savo_head/pan_tilt_state` (`JointState`), `/scan_state` (String), `/status` and `/diagnostics` (DiagnosticArray), `/camera/image_raw` and `/camera/camera_info`, `/apriltag/observations` (`savo_msgs/AprilTagObservation`), plus configured dashboard/detection summaries.

### Subscribed topics

`/savo_head/pan_tilt_cmd` (`Vector3`), `/scan_cmd` (String), camera image, and optional `/cmd_vel` and `/savo_localization/ready` confirmation gates.

### Services

`/savo_head/{start_scan,stop_scan,pause_scan,resume_scan,center,health_check}` use `std_srvs/srv/Trigger` where implemented/configured.

### Actions

| Action | Type | Purpose |
| --- | --- | --- |
| `/savo_head/apriltag/confirm` | `savo_msgs/action/ConfirmAprilTag` | Registration or arrival evidence |

## TF ownership

Dynamic `pantilt_mount_link -> pan_link -> tilt_link -> head_camera_link -> head_camera_optical_frame` (names configurable). `savo_description` must not duplicate these joints. Publishing requires calibrated transforms and fresh state.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| pan min/center/max | `0/72/170 deg` | Servo range |
| tilt min/center/max | `45/55/130 deg` | Servo range |
| PCA channels | `15/14` | Pan/tilt |
| watchdog | `0.50 s` | Stale head command |
| camera | `640x480@30` | Current profile |
| backend | `gstreamer_libcamerasrc` | Approved camera path |
| tag observations/quality/age | `5/0.70/0.50 s` | Confirmation gate |

## Launch files

`head_bringup`, `head_camera_ros`, `head_camera_stream`, `head_camera_stack`, and `head_apriltag` are production compositions; hardware/debug/fallback/scan-test launches are staged paths.

## Persistent state and runtime files

No database; calibration is configuration-controlled.

## Hardware ownership

Pan/tilt servos through PCA9685 and Core Pi Camera 2 NoIR. PCA9685 channels must not overlap base ownership.

## Dependencies

### Internal Robot Savo dependencies

`savo_msgs`, description frames, localization/motion status, mapping candidate workflow, named-location arrival confirmation.

### External ROS/system dependencies

OpenCV, libapriltag, gscam, GStreamer/libcamera, TF2, cv_bridge, I2C.

## Safety behavior

Angles clamp; stale commands stop/center per policy. Confirmation rejects stale, unstable, wrong-tag, moving, or unhealthy evidence. Head cannot authorize base motion.

## Failure and degraded behavior

Servo/backend failure prevents valid scan; camera loss prevents confirmation; invalid calibration suppresses dynamic TF.

## Startup and shutdown behavior

Centers on startup/shutdown. Streaming starts disabled and requires a host; automatic restart is disabled.

## Build

`bash deploy/core/build_core.sh --clean --test`

## Run

`ros2 launch savo_head head_bringup.launch.py`

## Validation and testing

Tests cover limits, calibration, scan, TF gates, camera health/launch, and AprilTag contracts/runtime.

## Current validation status

Implemented with earlier hardware evidence; current servo, camera, TF, scan, and tag integration need regression.

## Known limitations and remaining validation

Mount transforms/tag size and physical centers/limits must be measured and locked.

## Change-control considerations

Channels, limits, camera pipeline, frames, tag thresholds, and scan behavior are controlled hardware/interface changes.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_head/README.md)
- [Sensor mounting](../hardware/sensor_mounting.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Head test plan](../testing/head_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
