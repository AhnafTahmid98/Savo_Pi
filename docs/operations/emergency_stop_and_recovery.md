# Emergency Stop and Recovery

## Immediate emergency action

> [!WARNING]
> Protect people first. Software STOP is not a substitute for physical power
> isolation when motion is dangerous or software response is uncertain.

1. Use the installed physical emergency power/stop control immediately.
2. Warn people and prevent entry into the robot's path.
3. If software remains reachable and doing so does not delay physical action,
   send the Core external stop:

   ```bash
   ros2 topic pub --once /savo_control/external_stop std_msgs/msg/Bool \
     '{data: true}'
   ```

4. Also request control `STOP`:

   ```bash
   ros2 run savo_control mode_cmd_cli.py STOP
   ```

5. Cancel the active mission through its approved UI/bridge/operation owner.
6. If motion output persists, isolate motor/base power. Stop
   `savo_core.service` only after physical safety is established.

Do not diagnose before stopping. Do not clear the external stop or re-energize
the drivetrain during the immediate response.

## Post-stop recovery

1. Keep the robot isolated and preserve the scene when contact or near miss
   occurred.
2. Record time, mode/profile, active mission/map/location, operator, observed
   motion, STOP/cancel response, and emergency action.
3. Collect logs and state using [log collection](log_collection.md).
4. Inspect people/property, chassis, drivetrain, sensors, wiring, power, TF,
   localization, safety state, supervisor state, and command publishers.
5. Identify and correct the root cause; a restart alone is not correction.
6. Obtain the required maintainer/developer/safety review.
7. Re-run pre-operation and applicable regression checks with wheels raised or
   drivetrain isolated first.
8. Only after explicit release, clear the software latch:

   ```bash
   ros2 topic pub --once /savo_control/external_stop std_msgs/msg/Bool \
     '{data: false}'
   ros2 run savo_control mode_cmd_cli.py STOP
   ```

9. Confirm `STOP` and zero `/cmd_vel_safe` before restoring motion power.

Safety review is mandatory for uncommanded motion, collision/near miss, failure
to stop, emergency control failure, safety bypass, or unauthorized authority.

