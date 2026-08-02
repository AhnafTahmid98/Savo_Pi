# Base test plan

## Preconditions

Do not run a movement test until all of the following are true:

- physical geometry is measured, reviewed, and locked;
- the drivetrain wiring inspection is complete;
- the base battery is charged and its current/voltage telemetry is healthy;
- the physical E-stop is reachable and already tested;
- two operators are present;
- the robot is secured on rated stands with every wheel clear of the floor;
- the exclusion zone is clear;
- core bringup is running with control initially in `STOP`;
- LiDAR, perception safety, base watchdog, and control health are fresh.

Required hardware is the complete drivetrain, four encoder channels, base power
monitor, rated stands, and physical E-stop.

## Non-moving checks

From the repository root:

```bash
python3 tools/diag/motion/odom_test.py \
  --timeout 5 \
  --output log/diag/odom_stationary.json
```

Expected result: wheel odometry is available at a plausible rate and stationary
drift remains within the diagnostic threshold. Missing or stale odometry is
`BLOCKED`; invalid data or excessive drift is `FAIL`.

## Wheels-raised direction test

The operator must explicitly place the approved control system in `MANUAL`.
The diagnostic publishes only to `/cmd_vel_manual`; it never accesses motor
hardware directly and always sends five zero commands during cleanup.

Run one direction at a time:

```bash
python3 tools/diag/motion/motor_direction_test.py \
  --allow-motion \
  --wheels-raised \
  --direction forward \
  --speed 0.08 \
  --duration 0.5 \
  --output log/diag/motor_forward.json
```

Valid directions are:

```text
forward
backward
left
right
rotate_left
rotate_right
```

Repeat only after the previous result and physical wheel direction have been
reviewed. The accepted diagnostic bounds are 0.04–0.12 for speed and 0.1–1.0
seconds for duration.

Expected result: wheels follow the documented mecanum direction, safety remains
clear, releasing the command returns gated velocity to zero, and STOP remains
immediately available. Wrong direction, encoder loss, safety activation,
unexpected movement, or nonzero safe velocity after cleanup is `FAIL`.

## Odometry calibration recording

This diagnostic does not command motion. An operator performs one measured,
slow, approved movement while the tool records odometry:

```bash
python3 tools/diag/motion/odom_calibration.py \
  --allow-motion \
  --timeout 20 \
  --expected-distance-m 1.0 \
  --output log/diag/odom_one_metre.json
```

For an approved rotation, use `--expected-yaw-rad` instead. Compare measured
odometry with the independently measured physical distance or angle; do not
change calibration automatically from one run.

## Abort and cleanup

Abort immediately if a person enters the exclusion zone, the chassis shifts on
its stands, temperature/current exceeds the hardware limit, telemetry becomes
stale, a wheel behaves unexpectedly, or STOP is not immediate.

Cleanup:

1. engage STOP and, when needed, the physical E-stop;
2. verify `/cmd_vel_safe` is zero;
3. remove drivetrain power before touching hardware;
4. save JSON results, current logs, encoder plots, operator names, timestamps,
   and video;
5. mark the result `PASS`, `FAIL`, or `BLOCKED` honestly.
