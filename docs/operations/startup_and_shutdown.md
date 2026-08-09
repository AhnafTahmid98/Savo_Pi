# Startup and Shutdown

## Purpose

Routine procedure for an already installed and commissioned robot. Deployment,
unit installation, environment editing, and geometry release are maintainer or
developer work.

## Routine startup

> [!CAUTION]
> Startup is non-motion only while Core reports `safe_idle` and control reports
> `STOP`. Isolate motor power if those states cannot be observed reliably.

1. Complete the [pre-operation inspection](pre_operation_inspection.md).
2. Power Core and Edge with the installed controls; wait for the operating
   systems and network.
3. On Core, check the installed service:

   ```bash
   systemctl status savo_core.service --no-pager
   ```

4. If the installed unit is disabled or deliberately manual, start it on Core:

   ```bash
   sudo systemctl start savo_core.service
   ```

5. Verify Core state and STOP:

   ```bash
   ros2 topic echo --once /savo_bringup/core/state
   ros2 topic echo --once /savo_bringup/core/ready
   ros2 topic echo --once /savo_control/mode_state
   ros2 topic echo --once /safety/stop
   ```

6. On Edge, check or start `savo_edge.service`, then verify:

   ```bash
   systemctl status savo_edge.service --no-pager
   ros2 topic echo --once /savo_bringup/edge/state
   ros2 topic echo --once /savo_bringup/edge/ready
   ```

7. Verify distributed network/time, required sensor health, power, and the
   selected operation's readiness.
8. Start optional SavoMind, speech, UI, or observer only through the installed
   site configuration. Edge defaults speech/UI off; changing service environment
   is maintainer work.
9. Enter a motion mode only through its approved runbook.

The role units restart on failure. An unexpected restart during an operation is
an abort condition, not evidence of recovery.

## Routine shutdown

1. Cancel the active mapping/navigation operation through its owning control
   surface and wait for terminal acknowledgement.
2. Command and confirm `STOP`:

   ```bash
   ros2 run savo_control mode_cmd_cli.py STOP
   ros2 topic echo --once /savo_control/mode_state
   ros2 topic echo --once /cmd_vel_safe
   ```

3. Stop optional SavoMind/UI/observer processes.
4. On Edge:

   ```bash
   sudo systemctl stop savo_edge.service
   systemctl is-active savo_edge.service
   ```

5. On Core, after persistent operations are idle:

   ```bash
   sudo systemctl stop savo_core.service
   systemctl is-active savo_core.service
   ```

6. Confirm both units are inactive, no map/location write or release transaction
   is active, and required state files/logs have settled.
7. Use the operating system's controlled power-off command on each host:

   ```bash
   sudo poweroff
   ```

8. Remove physical power only after each host has halted.

## Abort and abnormal shutdown

Use [emergency stop](emergency_stop_and_recovery.md) for unexpected motion,
failed cancellation, electrical hazard, collision risk, or loss of software
control. After a forced power loss, do not resume until filesystem, map/location
state, supervisor state, batteries, and hardware are inspected and evidence is
collected.
