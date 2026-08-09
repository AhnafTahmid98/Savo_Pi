# savo_description

## Purpose

Authoritative URDF/Xacro, fixed-frame, physical-geometry, footprint, and geometry-identity package.

## Deployment

Built on Core and Edge and used by observer tooling. Core production runs `robot_state_publisher` through `description.launch.py`.

## Responsibilities

Define links/joints, fixed sensor transforms, wheel/chassis geometry, `measurement_state`, generated footprint, profile digest, and geometry validation/lock policy.

## Non-responsibilities and authority boundaries

Does not publish `map -> odom`, `odom -> base_footprint`, or dynamic head joints. It does not measure hardware automatically or authorize motion.

## Package structure

`urdf/`, `meshes/`, `rviz/`, geometry YAML, launch files, and validation/summary scripts.

## Runtime components

### `robot_state_publisher`

External ROS node launched with generated robot description; publishes only URDF-defined fixed/joint transforms. `joint_state_publisher_gui` is visualization-only.

## Runtime data flow

`geometry YAML + Xacro -> robot_description -> robot_state_publisher -> /tf_static`.

## ROS interfaces

### Published topics

`/robot_description` (`std_msgs/msg/String`, transient local) and `/tf_static` (`tf2_msgs/msg/TFMessage`) through `robot_state_publisher`.

### Subscribed topics

`/joint_states` only where URDF movable joints are driven; dynamic head TF remains `savo_head` authority.

### Services

No package-owned service.

### Actions

No action.

## TF ownership

Fixed robot tree including `base_footprint -> base_link` and static sensor/mount frames such as `base_link -> laser_frame`, IMU, RealSense, and head mount. Dynamic `map/odom/base` and head pan/tilt transforms are excluded.

## Parameters and configuration

`robot_dimensions.yaml`, `wheel_geometry.yaml`, and `sensor_mounts.yaml` form profile `robot_savo_core_v1`. The current repository status identifies geometry as provisional; `measurement_state`, digest, and generated footprint must agree before a locked profile is accepted.

## Launch files

`description.launch.py`/`rsp.launch.py` are runtime; `display`, `view_model`, and `tf_debug` are developer/observer tools.

## Persistent state and runtime files

No mutable state. Geometry/digest artifacts are version-controlled configuration and participate in map-release identity.

## Hardware ownership

None; describes hardware.

## Dependencies

### Internal Robot Savo dependencies

Consumed by localization, LiDAR, head, RealSense, perception, mapping, Nav2, bringup, and observer.

### External ROS/system dependencies

Xacro, URDF, `robot_state_publisher`, optional joint-state publisher/RViz.

## Safety behavior

Motion-capable bringup rejects provisional/unlocked or digest-inconsistent geometry. Fixed transforms must have one publisher.

## Failure and degraded behavior

Missing/invalid geometry blocks bringup/readiness; no guessed transform is substituted.

## Startup and shutdown behavior

Description is evaluated before dependent stacks; TF ends when publisher stops.

## Build

Role build scripts or `colcon build --packages-select savo_description`.

## Run

`ros2 launch savo_description description.launch.py`

## Validation and testing

Tests parse Xacro/config, enforce required/no-duplicate frames, geometry contract, install assets, and AM-0 profile rules.

## Current validation status

Source-validated; current physical geometry is not locked. Earlier hardware baseline does not validate this profile digest.

## Known limitations and remaining validation

Measure, review, lock, regenerate footprint/digest, then verify robot TF physically.

## Change-control considerations

Geometry/frame changes invalidate related footprint and may invalidate map release identity and require navigation regression.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_description/README.md)
- [Description test plan](../testing/description_test_plan.md)
- [Measurement checklist](../hardware/measurement_checklist.md)
- [Ownership matrix](package_ownership_matrix.md)
