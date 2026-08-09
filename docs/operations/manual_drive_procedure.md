# Manual Drive Procedure

## Purpose and classification

> [!WARNING]
> This motion-authorized procedure can move the robot in any mecanum direction.

Intended for a trained operator with a spotter and immediate physical stop
access. The installed keyboard node is the local manual source. It publishes
only `/cmd_vel_manual`; commands then pass through:

```text
keyboard manual source -> savo_control -> /cmd_vel
  -> savo_perception -> /cmd_vel_safe -> savo_base
```

## Preconditions

- Current-source motion regression and geometry approval cover this scope.
- Pre-operation inspection is PASS; area and stop control are ready.
- Core is healthy in `safe_idle`, control is `STOP`, safety is current/clear,
  and no mapping/navigation/recovery command source is active.
- Supervisor permission has been explicitly verified through the deployed
  authority surface. The direct mode topic does not itself request supervisor
  authorization; do not treat mode acceptance as mission permission.
- A separate terminal is available on Core with the installed overlay sourced.

## Procedure

1. Start the manual publisher while still in `STOP`:

   ```bash
   ros2 run savo_control keyboard_teleop_node.py
   ```

2. In another Core terminal, enter `MANUAL` and verify the selected mode:

   ```bash
   ros2 run savo_control mode_cmd_cli.py MANUAL
   ros2 topic echo --once /savo_control/mode_state
   ```

3. Use the keys printed by the node:

   | Motion | Key |
   | --- | --- |
   | Forward / backward | `w` / `s` |
   | Strafe left / right | `a` / `d` |
   | Rotate counter-clockwise / clockwise | `q` / `e` |
   | Stop command | `x` or Space |
   | Linear speed up/down | `t` / `g` |
   | Angular speed up/down | `y` / `h` |
   | Reset speeds / help | `r` / `p` |

4. Begin with a brief wheels-raised or minimum-speed command. Verify direction,
   safety response, and spotter communication before floor travel.
5. Keep the terminal focused and send movement keys continuously. The keyboard
   source zeros after `0.50 s` without input; the mux/shaper timeout is `0.35 s`
   and base watchdog is `0.30 s`.
6. Finish with `x`, request `STOP`, verify it, then use `Ctrl+C`:

   ```bash
   ros2 run savo_control mode_cmd_cli.py STOP
   ros2 topic echo --once /savo_control/mode_state
   ```

## Configured envelope and behavior

Keyboard defaults are `0.12 m/s` and `0.35 rad/s`; its configured maxima are
`0.25 m/s` and `0.60 rad/s`. The downstream shaper further caps x/y/yaw at
`0.20 m/s`, `0.18 m/s`, and `0.55 rad/s`. These are configuration limits, not
validated safe speeds. The perception gate can zero or constrain the command.

## Abort conditions

Immediately stop for unexpected direction/speed, loss of stop access, stale or
failed safety sensing, unexpected mode/source, duplicate publisher, obstacle
gate failure, power fault, service restart, poor traction, person/property
risk, or failure of keyboard release to zero motion. Use the emergency runbook
if normal STOP is not immediate and confirmed.

