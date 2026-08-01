# Phase 3 real-robot validation

This procedure validates the Phase 3 startup and edge authority on the real two-Pi Robot Savo. It intentionally separates non-motion checks from controlled movement checks.

## Preconditions

- `savo-core` and `savo-edge` use the production direct-Ethernet ROS 2 network.
- The physical emergency stop and the software safety stop are available to the operator.
- Wheels are initially lifted or the robot is inside a clear test area.
- `savo_bridge`, `savo_base`, `savo_control`, `savo_perception`, `savo_lidar`, `savo_localization`, `savo_power`, `savo_mapping`, `savo_nav`, `savo_head`, and `savo_locations` use the tested package configurations.
- The supervisor systemd service starts disarmed (`SAVO_SUPERVISOR_AUTO_ARM=false`).

## Stage A — non-motion startup preflight

Run on `savo-core`:

```bash
ros2 run savo_supervisor real_robot_preflight.sh
```

Pass criteria:

1. `/savo_supervisor/manage_system_state` exists.
2. `/savo_supervisor/authorize_operation` exists.
3. Core, bridge and safety inputs are present.
4. The state reports `READY_TO_ARM` with `system_armed=false`.
5. A motion operation is denied with `system_not_armed`.

Do not arm until every required core and bridge dependency is truthful.

## Stage B — explicit arm and safe idle

Use the operator terminal:

```bash
ros2 service call /savo_supervisor/manage_system_state \
  savo_msgs/srv/ManageSystemState \
  "{command: 1, request_id: 'real-arm-1', actor_id: 'system_operator', reason: 'real_robot_validation', expected_generation: 0}"
```

Pass criteria:

- `system_state=ARMED` or `ARMED_DEGRADED`.
- `/savo_supervisor/system_ready` is `true`.
- The robot remains stationary until a separate guarded movement command is sent.

## Stage C — bridge/network loss

While the robot is stationary, stop `savo_bridge` or disconnect the core-edge Ethernet link.

Pass criteria:

- `remote_commands_ready=false`.
- Any active remote-origin mission becomes `REVOKED`.
- Local core safety remains observable.
- The supervisor process remains alive.
- Restoring the bridge does not automatically resume the mission.

Resume requires a matching owner, request ID and current generation.

## Stage D — safety stop during controlled motion

In a clear area, acquire a low-speed manual-control lease and command only the previously hardware-tested low-speed movement profile. Trigger `/safety/stop` using a real obstacle or the physical safety test procedure.

Pass criteria:

- Motion stops through the existing `savo_control`/`savo_base` safety path.
- The active mission becomes `REVOKED`.
- The supervisor remains `RUNNING` and reports `ESTOP`/`STOPPED` rather than crashing.
- Clearing the obstacle does not automatically resume motion.

## Stage E — required core process fault and latch persistence

With the robot stationary and disarmed from movement, stop one required core dependency, such as the LiDAR health publisher.

Pass criteria:

- The supervisor disarms.
- `fault_latched=true` and the system reports `FAULT_LATCHED`.
- Restarting `savo-supervisor.service` preserves the fault latch.
- Restoring the dependency alone does not clear the latch.
- `CLEAR_FAULT_LATCH` succeeds only when core health, known safety, startup dependencies and mission-idle conditions are all true.

## Stage F — controlled shutdown intent

Request controlled shutdown:

```bash
ros2 run savo_supervisor request_controlled_shutdown.sh real_robot_validation_complete
```

Pass criteria:

- `shutdown_requested=true`.
- Operating mode becomes `SHUTTING_DOWN`.
- New missions are rejected.
- The supervisor publishes shutdown intent but does not directly invoke `poweroff`.
- The platform shutdown owner performs the actual operating-system shutdown after its own checks.

## Evidence to save

Save these outputs with timestamps:

```bash
journalctl -u savo-supervisor.service --since today --no-pager
ros2 topic echo --once /savo_supervisor/state_summary
ros2 topic echo --once /savo_supervisor/health
ros2 topic echo /savo_supervisor/events
cat /var/lib/robot_savo/supervisor/system_state.json
```

Phase 3 is real-hardware validated only after Stages A–F pass on the physical robot.
