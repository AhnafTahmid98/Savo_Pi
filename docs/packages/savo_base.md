# savo_base

## Purpose

`savo_base` is the Core-side drivetrain execution package and the only package permitted to translate an accepted velocity command into PCA9685 motor-board writes.

## Deployment

Built and run on `savo-core`. `savo_bringup` includes `base_bringup.launch.py`; bench launches provide hardware-only, safe-idle, dry-run, wheel-diagnostic, and teleoperation profiles.

## Responsibilities

- Consume only the safety-gated `/cmd_vel_safe` command.
- Apply final stop, slowdown, watchdog, mecanum mixing, normalization, inversion, and PWM limits.
- Quench direction reversals and stop outputs on stale commands or board errors.
- Publish drivetrain/watchdog state and diagnostic summaries.

## Non-responsibilities and authority boundaries

The package does not select missions, arbitrate command sources, plan paths, or decide environmental safety. No upstream node may access the motor board or bypass `/cmd_vel_safe`.

## Package structure

Production C++ is under `src/` and `include/`; Python nodes are monitoring/fallback utilities. `config/` defines the board, kinematics, topics, and timeout policy; `scripts/` contains operator diagnostics.

## Runtime components

### `base_driver_node`

Production C++ node. It mixes normalized `vx`, `vy`, and `wz` into FL/RL/FR/RR commands, applies inversion and output caps, and writes the selected motor-board backend. `dryrun` is for non-hardware validation.

### Monitoring and diagnostic nodes

`base_watchdog_node.py`, `base_state_publisher_node.py`, `base_heartbeat_node.py`, and `base_diag_runner_node.py` monitor freshness, aggregate state, publish heartbeats, and run bounded diagnostics. They do not replace the driver's internal watchdog. CLI scripts are operator tools, not production command authorities.

## Runtime data flow

```text
/cmd_vel_safe + /safety/stop + /safety/slowdown_factor
                         -> base_driver_node -> PCA9685 -> four motors
                         -> state/watchdog/diagnostic topics
```

## ROS interfaces

### Published topics

| Topic | Type | Purpose |
| ----- | ---- | ------- |
| `/savo_base/watchdog_state` | `std_msgs/msg/String` | Driver/watchdog freshness state |
| `/savo_base/base_state` | `std_msgs/msg/String` | Driver and board execution state |
| `/savo_base/state_summary` | `std_msgs/msg/String` | Aggregated base status |
| `/savo_base/state_heartbeat`, `/savo_base/heartbeat` | `std_msgs/msg/String`, `std_msgs/msg/Bool` | Liveness outputs |
| `/savo_base/watchdog_trip` | `std_msgs/msg/Bool` | Debounced watchdog trip |
| `/savo_base/diag/state`, `/savo_base/diag/event`, `/savo_base/diag/busy` | String/string/bool | Diagnostic runner state |

### Subscribed topics

| Topic | Type | Purpose |
| ----- | ---- | ------- |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Sole motion input |
| `/safety/stop` | `std_msgs/msg/Bool` | Independent stop gate |
| `/safety/slowdown_factor` | `std_msgs/msg/Float32` | Bounded `[0,1]` scaling |

Monitoring nodes additionally observe base/watchdog outputs and diagnostic request topics.

### Services

No production ROS service is exposed.

### Actions

This package exposes no action interface.

## TF ownership

None. Base geometry is static in `savo_description`; `odom -> base_footprint` is owned by localization.

## Parameters and configuration

| Parameter | Default | Purpose / constraint |
| --------- | ------: | -------------------- |
| `cmd_topic` | `/cmd_vel_safe` | Must not be changed to an ungated topic |
| `loop_hz` | `30.0` | Motor execution rate |
| `watchdog_timeout_s` | `0.30` | Zeros output after command loss |
| `max_duty` | `3500` | Robot cap below PCA9685 maximum 4095 |
| `i2c_bus`, `pca9685_addr` | `1`, `64` | Motor-board connection |
| `quench_ms` | `18` | Direction reversal protection |
| `invert_fl/rl/fr/rr` | `true` | Current wiring polarity; change only after direction test |
| `wheel_diameter_m` | `0.065` | Kinematic configuration |
| `wheelbase_m`, `track_width_m` | `0.160`, `0.216` | Owner-measured wheel-center geometry |
| `kinematic_k_m` | `0.188` | Synchronized fallback for `(wheelbase + track) / 2` |
| `board_backend` | `auto` | Production hardware selection; tests may use `dryrun` |

Encoder values in `mecanum_kinematics.yaml` describe the relationship but acquisition/odometry are owned by `savo_localization`.

## Launch files

| Launch file | Purpose | Important notes |
| ----------- | ------- | --------------- |
| `base_bringup.launch.py` | Production driver plus state/watchdog diagnostics | C++ driver is default |
| `base_safe_idle.launch.py` | Initialize without motion authority | Commands remain zero |
| `base_hw_only.launch.py` | Isolated driver | Commissioning only |
| `base_dryrun_test.launch.py` | No-hardware backend | Not evidence of physical polarity |
| `base_diag_board.launch.py`, `base_diag_wheels.launch.py`, `base_teleop_test.launch.py` | Staged diagnostics | Require physical safety procedure |

## Persistent state and runtime files

No application database is owned. ROS logs are external runtime artifacts.

## Hardware ownership

Owns the Freenove mecanum motor interface through PCA9685 on I2C bus 1, address `0x40`, with channel mapping in `motor_board_freenove.yaml`. It does not own GPIO encoders.

## Dependencies

### Internal Robot Savo dependencies

Consumes safety output from `savo_perception`, the shaped command from `savo_control`, and geometry/configuration coordinated with `savo_description`.

### External ROS/system dependencies

ROS 2 `rclcpp`, geometry/std/diagnostic messages, Linux I2C, and Core `liblgpio` support.

## Safety behavior

The driver fails closed: safety stop, stale command, invalid slowdown, backend initialization/write failure, or shutdown causes zero motor output. Its internal watchdog remains authoritative if monitoring fails.

## Failure and degraded behavior

Hardware backend failure prevents motion; dry-run is an explicit diagnostic substitution, not silent degradation. Loss of state publishers does not grant authority; loss of the driver stops writes.

## Startup and shutdown behavior

Startup initializes the board at zero. Production control starts in `STOP`. Shutdown and exceptions request a board-wide stop.

## Build

`bash deploy/core/build_core.sh --clean --test`

## Run

`ros2 launch savo_base base_bringup.launch.py`

## Validation and testing

Tests cover mixing, clamping/scaling, wheel commands, topics, dry-run behavior, and watchdog logic. Use the staged wheels-raised procedure before floor motion.

## Current validation status

Implemented with retained PC-test and earlier physical-baseline evidence. Current-source polarity, watchdog, STOP, stale-command, and direction behavior require hardware regression.

## Known limitations and remaining validation

Command space is normalized; configured physical limits are not the active driver limit model. Measured wheel-center geometry is synchronized, while loaded radius, polarity, encoder counts, and real-floor kinematics still require regression.

A retained Python fallback model docstring still mentions `savo_intent`; the production C++ driver does not depend on that retired package or topic.

## Change-control considerations

Changes to input topic, watchdog, inversion, channels, PWM cap, I2C address, or geometry are safety-critical.

## Related documentation

- [Package implementation README](../../savo_ws/src/core/savo_base/README.md)
- [Motion authority model](../architecture/motion_authority_model.md)
- [Base test plan](../testing/base_test_plan.md)
- [Package ownership matrix](package_ownership_matrix.md)
- [Current system status](../status/current_system_status.md)
