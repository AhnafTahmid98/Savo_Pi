# savo_observer

## Purpose

Read-only operator-workstation telemetry, browser dashboard, RViz, and connection diagnostics.

## Deployment

Operator ROS 2 Jazzy workstation only; absent from both Pi role arrays.

## Responsibilities

Subscribe/aggregate telemetry with freshness, render browser/dashboard views, launch constrained RViz profiles, and check network/domain connectivity.

## Non-responsibilities and authority boundaries

No motion, mission, map/location approval, initial-pose, goal, teleop, service client, action client, or write authority.

## Package structure

C++ telemetry/dashboard nodes, browser assets, RViz/config profiles, launch/scripts, and read-only validators.

## Runtime components

### `observer_telemetry_node`

Creates bounded read-only snapshots and freshness state.

### `observer_dashboard_node`

Serves/formats browser dashboard data without mutation endpoints. RViz is external and launched with restricted tools.

## Runtime data flow

`robot DDS telemetry -> freshness/snapshot -> dashboard/RViz`.

## ROS interfaces

### Published topics

Observer-local telemetry snapshot/status topics configured in `observer.yaml`/`telemetry.yaml`; no robot command topics.

### Subscribed topics

Configured robot status, control, safety, nav, mapping, localization, sensor, speech, power, and TF/map visualization streams.

### Services

None.

### Actions

None.

## TF ownership

None; visualizes TF.

## Parameters and configuration

Topics, stale thresholds, layout, bandwidth/profile, bind address/port, and RViz config are explicit assets. Stale data remains labeled rather than shown as current.

## Launch files

`observer.launch.py` is primary; `full_observer`, `dashboard_observer`, `rviz_observer`, and `observer_bridge` select surfaces.

## Persistent state and runtime files

Browser/RViz preferences/logs only; no robot state.

## Hardware ownership

Operator workstation display/network only.

## Dependencies

### Internal Robot Savo dependencies

Shared descriptions/messages and remote telemetry producers.

### External ROS/system dependencies

RViz2, browser/web serving, ROS networking.

## Safety behavior

Source validator rejects clients/services/actions, command topics, and unsafe RViz tools including SetGoal/SetInitialPose/Teleop.

## Failure and degraded behavior

Network/topic loss marks views stale; robot operation authority is unchanged.

## Startup and shutdown behavior

Starts after workstation ROS/network setup; stopping disconnects presentation only.

## Build

`colcon build --packages-up-to savo_observer --symlink-install`.

## Run

`ros2 launch savo_observer observer.launch.py`.

## Validation and testing

`bash deploy/observer/validate_observer.sh`; C++ freshness/snapshot tests and launch/dashboard/RViz/read-only contracts.

## Current validation status

Source-validated; actual workstation build and production-network connection required.

## Known limitations and remaining validation

Bandwidth, browser compatibility, DDS discovery, and display layouts need workstation integration.

## Change-control considerations

Any mutation tool/client is an architecture violation and requires explicit authority review, not a normal observer change.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_observer/README.md)
- [Network architecture](../architecture/network_architecture.md)
- [Observer test plan](../testing/observer_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
- [Current status](../status/current_system_status.md)
