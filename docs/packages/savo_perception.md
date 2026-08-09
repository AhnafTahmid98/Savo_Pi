# savo_perception

## Purpose

Role-specific sensing package: Core owns near-field sensors and the final velocity safety gate; Edge optionally produces a filtered D435 obstacle cloud.

## Deployment

Built on both Pis. Core runs range/safety/gate nodes. Edge runs `obstacle_cloud_filter_node` only when explicitly enabled; it defaults disabled pending physical validation.

## Responsibilities

Core: VL53L1X mux and ultrasonic acquisition, range health, STOP/slowdown fusion, `/cmd_vel -> /cmd_vel_safe`. Edge: finite/range/height/robot-self filtering, cloud freshness/status/heartbeat.

## Non-responsibilities and authority boundaries

Does not select desired motion, plan paths, write motors, own D435 acquisition, or own VO. It may constrain/stop but never increase authority.

## Package structure

Production C++ sensor/gate/cloud nodes; retained Python fallbacks and diagnostic CLIs; role launches/config.

## Runtime components

### Core

`vl53_mux_node`, `ultrasonic_node`, `safety_stop_node`, `cmd_vel_safety_gate`, `range_health_node`, and Python `sensor_dashboard_node`. C++ is production.

### Edge

`obstacle_cloud_filter_node` consumes raw D435 cloud and publishes obstacle-only points for the guarded Nav2 voxel profile.

## Runtime data flow

```text
ToF + ultrasonic + front depth -> safety_stop_node -> stop/slowdown
/cmd_vel + safety state -> cmd_vel_safety_gate -> /cmd_vel_safe -> base
D435 raw cloud -> optional filter -> /savo_perception/obstacles/points -> Nav2
```

## ROS interfaces

### Published topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/savo_perception/range/{left_m,right_m,front_ultrasonic_m}` | `std_msgs/msg/Float32` | Core ranges |
| `/safety/stop`, `/safety/slowdown_factor` | `Bool`, `Float32` | Independent safety state |
| `/savo_perception/safety_state`, `/range_health`, `/sensor_status`, `/heartbeat` | Status | Health/readiness |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Sole base input |
| `/savo_perception/obstacles/points` | `sensor_msgs/msg/PointCloud2` | Optional obstacle-only cloud |
| `/savo_perception/obstacle_cloud/{health,status,heartbeat}` | Status | Edge cloud gate |

### Subscribed topics

`/cmd_vel`, Core range topics, `/depth/min_front_m`; Edge raw `/camera/camera/depth/color/points`; diagnostic request/cancel topics.

### Services

No public production service.

### Actions

No action.

## TF ownership

None; requires source cloud frames and static sensor TF from description.

## Parameters and configuration

Safety thresholds, stale policy, TCA9548A/ToF and ultrasonic GPIO/timing, gate command timeout, and cloud crop/self-filter bounds are in node/config sources. Treat hardware thresholds/bounds as provisional until measured; no value is inferred here.

## Launch files

Core `perception_bringup`, `range_sensors`, `safety_bringup`; Edge `obstacle_cloud_filter`. Production bringup selects by role.

## Persistent state and runtime files

None.

## Hardware ownership

Core VL53L1X sensors through TCA9548A and HC-SR04 ultrasonic hardware. Edge component consumes, but does not own, RealSense D435.

## Dependencies

### Internal Robot Savo dependencies

Control input, base consumer, realsense cloud/depth, description TF, Nav2 optional cloud, supervisor/bringup/UI observations.

### External ROS/system dependencies

Linux I2C/GPIO, PCL/point-cloud and standard ROS messages.

## Safety behavior

Core gate is fail-closed on stale/invalid required range state or command timeout and cannot be bypassed by Nav2, UI, bridge, or SavoMind. Slowdown is clamped. Edge cloud is not a substitute for near-field gate.

## Failure and degraded behavior

Core sensor/gate loss must stop motion or remove readiness. Optional Edge cloud failure disables the voxel profile and falls back to validated LiDAR-only navigation, not unsafe cloud reuse.

## Startup and shutdown behavior

Safety state initializes conservatively; gate emits zero until valid command/safety state. Cloud remains opt-in.

## Build

Use role build scripts.

## Run

`ros2 launch savo_perception perception_bringup.launch.py` (Core) or `obstacle_cloud_filter.launch.py` (Edge validation).

## Validation and testing

Tests cover sensors/models, safety fusion/gate, stale policy, topics, and synthetic cloud filtering/self-removal.

## Current validation status

Core path implemented/source-tested with earlier hardware baseline; current thresholds/stale behavior need hardware regression. Edge cloud is synthetic/source-validated only and blocked from production voxel use.

## Known limitations and remaining validation

Measure sensor mounts/thresholds and D435 self-filter bounds; validate clearing/freshness and real robot occlusion before voxel enablement.

`config/topics.yaml` still lists retired `savo_dashboard` as a descriptive safety-output consumer. No such package or production role member exists; current consumers are the read-only UI/observer and other explicitly launched status clients. This metadata should be cleaned separately without changing the safety gate.

## Change-control considerations

Gate topics, stale defaults, sensor thresholds/GPIO/I2C, and cloud bounds/profile gate are safety-critical.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_perception/README.md)
- [Perception architecture](../architecture/perception_architecture.md)
- [Perception test plan](../testing/perception_test_plan.md)
- [Safety architecture](../architecture/safety_architecture.md)
