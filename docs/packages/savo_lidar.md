# savo_lidar

## Purpose

Owns the Core RPLIDAR A1 serial device and LaserScan contract for mapping and Nav2.

## Deployment

Core only; production uses the C++ driver via `lidar_bringup.launch.py` or `lidar_mapping_ready.launch.py`.

## Responsibilities

Serial acquisition, scan/filter/sector output, watchdog, health/rate/quality, and reconnect policy.

## Non-responsibilities and authority boundaries

Does not own static TF, fuse safety, stop motors, build maps, or plan paths.

## Package structure

C++ production driver; Python fallback/dry-run, filter, watchdog, health, state, and CLI utilities.

## Runtime components

### Nodes

`lidar_driver_node` is authoritative acquisition. `lidar_filter_node.py`, `lidar_watchdog_node.py`, `lidar_health_node.py`, and `lidar_state_publisher_node.py` process/monitor it. `lidar_py_driver_node.py` is not primary production.

## Runtime data flow

`RPLIDAR -> /scan -> mapping/Nav2`, with optional `/scan_filtered`, sector, and status outputs.

## ROS interfaces

### Published topics

`/scan`, `/scan_filtered`, `/savo_lidar/sector_scan/{front,left,right,back}` (`LaserScan`); `/savo_lidar/{state,health,watchdog_state,state_summary,scan_quality,heartbeat}` (status/liveness types defined by nodes/config).

### Subscribed topics

Filter/monitor nodes consume `/scan` and `/scan_filtered`; hardware driver has no ROS sensor input.

### Services

No public service.

### Actions

No action.

## TF ownership

Uses frame `laser_frame`; `savo_description` owns the static base transform.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| serial port/rate | `/dev/ttyUSB0`, `115200` | A1 ownership |
| expected rate | `5.5 Hz` | Health gate |
| range | `0.15–12.0 m` | Accepted interval |
| read timeout | `2.0 s` | Stale bound |
| reconnect | `true`, `0.5–5 s` | Retry policy |

## Launch files

Production: `lidar_bringup`, `lidar_mapping_ready`. Staged: hardware-only, dry-run, port diagnostic, filter test, health test.

## Persistent state and runtime files

None.

## Hardware ownership

Exclusive RPLIDAR A1 serial/motor lifecycle.

## Dependencies

### Internal Robot Savo dependencies

Description frame; consumers are mapping, navigation, readiness, supervisor, observer.

### External ROS/system dependencies

ROS sensor/std/diagnostic messages and serial access.

## Safety behavior

Stale/invalid scans report unhealthy and close downstream readiness. LiDAR itself never grants motion.

## Failure and degraded behavior

Read failure stops/reconnects and marks stale; mapping/navigation must not remain ready.

## Startup and shutdown behavior

Starts scanner after initialization/settle and stops motor/serial on shutdown.

## Build

`bash deploy/core/build_core.sh --clean --test`

## Run

`ros2 launch savo_lidar lidar_mapping_ready.launch.py`

## Validation and testing

Tests cover dry-run, filters, frames/QoS, rate/range/angle, watchdog, sectors, stale policy.

## Current validation status

Implemented/source-tested with earlier hardware baseline; current live scan/reconnect/mapping-ready regression required.

## Known limitations and remaining validation

`/dev/ttyUSB0` is not persistent identity; mounting/angle offset need locked-geometry verification.

## Change-control considerations

Device identity, frame, offset/inversion, filters, and stale/reconnect limits affect localization/safety.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_lidar/README.md)
- [Perception architecture](../architecture/perception_architecture.md)
- [LiDAR test plan](../testing/lidar_test_plan.md)
- [Full robot test plan](../testing/full_robot_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
