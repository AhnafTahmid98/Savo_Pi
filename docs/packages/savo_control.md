# savo_control

## Purpose

Core command-mode, velocity arbitration, shaping, bounded maneuver, stuck-detection, and recovery package.

## Deployment

Core build/runtime set; `savo_bringup` starts `control_bringup.launch.py` with `STOP` as the deployment default.

## Responsibilities

Manage `STOP`, `MANUAL`, `AUTO`, `NAV`, and `RECOVERY`; select `/cmd_vel_manual`, `/cmd_vel_auto`, `/cmd_vel_nav`, or `/cmd_vel_recovery`; shape the selected command into `/cmd_vel`; provide bounded rotation/approach and guarded recovery.

## Non-responsibilities and authority boundaries

It does not authorize missions, decide obstacle safety, publish `/cmd_vel_safe`, plan Nav2 paths, or write motors. Perception and base remain downstream independent authorities.

## Package structure

Production controllers/nodes are C++ under `src/`/`include/`. Python executables are dashboards, teleop, test managers, status helpers, and diagnostics. YAML files define runtime authority and limits.

## Runtime components

### Production command path

`control_mode_manager_node` owns mode; `twist_mux_node` selects a permitted fresh lane; `cmd_vel_shaper_node` applies deadband/slew/speed limits. `stuck_detector_node`, `recovery_manager_node`, and `backup_escape_node` detect/execute guarded recovery.

### Bounded and diagnostic components

`rotate_to_heading_node` serves the typed rotation action; `distance_approach_node` uses front depth on AUTO. `heading_pid_node`, `velocity_test_pattern_node`, Python PID/test nodes, keyboard teleop, dashboards, and CLIs are commissioning paths.

## Runtime data flow

`approved lane -> /cmd_vel_mux -> shaper -> /cmd_vel -> savo_perception -> /cmd_vel_safe -> savo_base`.

## ROS interfaces

### Published topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/savo_control/mode_state`, `/mode_reason`, `/control_status` | `std_msgs/msg/String` | Authority state |
| `/cmd_vel_mux`, `/cmd_vel`, `/cmd_vel_auto`, `/cmd_vel_recovery` | `geometry_msgs/msg/Twist` | Selected/shaped or bounded lane commands |
| `/savo_control/twist_mux/source`, `/twist_mux/status`, `/cmd_vel_shaper/status` | `std_msgs/msg/String` | Routing/shaping state |
| `/savo_control/stuck_state`, `/stuck_detected`, `/recovery_request`, `/recovery_active`, `/recovery_state`, `/recovery_status` | String/bool | Recovery state |
| `/savo_control/rotate_state`, `/rotate_status`, `/distance_approach_status` | `std_msgs/msg/String` | Maneuver state |

### Subscribed topics

`/savo_control/mode_cmd`, `/savo_control/external_stop`, `/savo_control/manual_override`; the four command lanes; `/safety/stop`, `/safety/slowdown_factor`; `/cmd_vel_safe`, `/odometry/filtered`; left/right ranges and `/depth/min_front_m`. The mode manager republishes authoritative external-stop state at 20 Hz on `/savo_control/external_stop_state` for fresh cross-host observation.

### Services

No public production service.

### Actions

| Action | Type | Purpose |
| --- | --- | --- |
| `/savo_control/rotate_to_heading` (configurable) | `savo_msgs/action/RotateToHeading` | Bounded absolute-yaw rotation through AUTO |

## TF ownership

None; consumes odometry.

## Parameters and configuration

| Parameter | Default | Purpose / constraint |
| --- | ---: | --- |
| `startup_mode`, `fallback_mode` | `STOP` | Fail-closed authority |
| mux/shaper `command_timeout_s` | `0.35` | Zero stale input |
| shaper `max_vx/max_vy/max_wz` | `0.20/0.18/0.55` | Active caps |
| `external_stop_forces_stop` | `true` | Immediate STOP |
| stuck `stuck_time_s` | `2.0` | No-progress interval |
| stuck `publish_recovery_request` | `false` | Automatic recovery remains disabled pending tuning |
| rotate tolerance/timeout | `0.06 rad/8 s` | Rotation bound |
| approach target/hard minimum | `0.60/0.35 m` | Depth maneuver bound |

## Launch files

`control_bringup.launch.py` is production. `teleop_control`, `rotate_test`, `distance_approach`, `recovery_test`, `heading_pid_test`, and `auto_test_control` launches are explicit tests.

## Persistent state and runtime files

None beyond logs.

## Hardware ownership

None.

## Dependencies

### Internal Robot Savo dependencies

`savo_msgs`, localization odometry, perception ranges/safety, Nav2 output, supervisor authorization context, and base downstream execution.

### External ROS/system dependencies

`rclcpp`, actions, geometry/nav/std messages, TF2.

## Safety behavior

Unknown modes, stale/missing active lanes, external stop, and timeouts produce zero/STOP. Recovery cannot bypass the mux, perception gate, or base watchdog.

## Failure and degraded behavior

Source loss zeros the mux; mode-manager loss removes a valid route; shaper loss removes `/cmd_vel`. Stale odometry suppresses stuck declarations and blocks recovery.

## Startup and shutdown behavior

Starts/falls back to `STOP`, publishes zero while inactive, and resets shaping on timeout.

## Build

`bash deploy/core/build_core.sh --clean --test`

## Run

`ros2 launch savo_control control_bringup.launch.py startup_mode:=STOP`

## Validation and testing

Tests cover modes, source selection, limiters/PIDs, action runtime, recovery, launch/config contracts, and Python helper separation.

## Current validation status

Implemented/source-tested with earlier robot baseline evidence; current target and integrated arbitration/recovery regression required.

## Known limitations and remaining validation

Automatic stuck recovery is off. Limits, PID, approach, and recovery timing require physical tuning.

## Change-control considerations

Modes, priorities, source topics, timeout/zero policy, limits, and recovery enablement are motion-safety changes.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_control/README.md)
- [Motion authority model](../architecture/motion_authority_model.md)
- [Control test plan](../testing/control_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
