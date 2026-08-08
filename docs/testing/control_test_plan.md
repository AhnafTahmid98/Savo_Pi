# Savo Control Test Plan

## Objective

Validate that `savo_control` owns command selection and shaping correctly, starts in `STOP`, routes only the authorized command source, responds to external safety stops, and coordinates bounded recovery without bypassing `savo_perception` or `savo_base`.

This plan separates source/PC validation from physical motion. Complete all non-motion stages before raising the robot's wheels or placing it on the floor.

## Scope

Components under test:

- control mode manager;
- velocity-source mux;
- command shaper;
- external stop handling;
- stuck detector;
- recovery manager and backup escape;
- distance approach;
- rotate-to-heading action;
- control status and diagnostics;
- integration with perception and base command paths.

## Production command chain

```text
/cmd_vel_manual ─┐
/cmd_vel_auto   ─┼─> mode/mux ─> /cmd_vel_mux ─> shaper ─> /cmd_vel
/cmd_vel_nav    ─┤                                      │
/cmd_vel_recovery┘                                      ▼
                                      savo_perception safety gate
                                                        │
                                                 /cmd_vel_safe
                                                        │
                                                    savo_base

```

No test may publish directly to motor hardware. Publishing `/cmd_vel_safe` is permitted only in the separate base test plan with the drivetrain physically controlled.

## Required equipment

- Core Pi with the current tested repository revision;
- robot or hardware bench with emergency stop;
- blocks/stand capable of lifting all four wheels for motion stages;
- operator at the emergency stop;
- second observer for floor-motion stages;
- ROS 2 Jazzy workstation or Core terminal;
- retained test log location.

## Preconditions

- Core role clean build and tests pass.
- Geometry and direction conventions are known.
- Base and near-field safety packages are available for integration stages.
- No duplicate control nodes are running.
- Robot mode is `safe_idle` and control mode is `STOP`.
- Floor-motion stages remain blocked until base, encoder, IMU, and perception component tests pass.

## Stage C0 — source and package tests

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_control --symlink-install
source install/setup.bash
colcon test --packages-select savo_control --ctest-args --output-on-failure
colcon test-result --verbose

```

Pass criteria:

- build succeeds;
- zero test failures and errors;
- launch/config/source-contract tests pass;
- rotate action and recovery contracts pass;
- no production code path requires a Python-only motor authority implementation.

## Stage C1 — configuration review

Review:

```bash
cd ~/Savo_Pi/savo_ws/src/core/savo_control
sed -n '1,240p' config/control_mode_manager.yaml
sed -n '1,240p' config/twist_mux.yaml
sed -n '1,240p' config/cmd_vel_shaper.yaml
sed -n '1,260p' config/recovery.yaml

```

Confirm:

- startup mode is `STOP`;
- source priorities match the approved authority model;
- command timeout is finite;
- velocity and acceleration limits are conservative for the current test stage;
- recovery commands use `/cmd_vel_recovery` rather than bypassing the mux;
- external stop is `/savo_control/external_stop`;
- output remains upstream of the perception gate.
Record every non-default parameter used in testing.

## Stage C2 — canonical bringup without motors

Disable or physically isolate drivetrain output. Launch the package:

```bash
ros2 launch savo_control control_bringup.launch.py

```

In another shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/savo_ws/install/setup.bash
ros2 node list
ros2 topic list | grep -E 'cmd_vel|savo_control|safety'

```

Verify the expected nodes and that `/savo_control/mode_state` reports `STOP`.

```bash
ros2 topic echo /savo_control/mode_state --once
ros2 topic echo /savo_control/mode_reason --once

```

Pass criteria:

- all required nodes start once;
- no command is emitted at startup;
- state is current and reports `STOP`;
- no direct publisher to `/cmd_vel_safe` is created by `savo_control`;
- clean SIGINT shutdown completes.

## Stage C3 — mode command validation

List and dry-run modes:

```bash
ros2 run savo_control mode_cmd_cli.py --list
ros2 run savo_control mode_cmd_cli.py STOP --dry-run --json

```

With drivetrain isolated, exercise valid transitions one at a time:

```bash
ros2 run savo_control mode_cmd_cli.py MANUAL
ros2 topic echo /savo_control/mode_state --once
ros2 run savo_control mode_cmd_cli.py STOP

```

Repeat for `AUTO`, `NAV`, and `RECOVERY` only when the runtime contract permits the transition.

Negative tests:

```bash
ros2 run savo_control mode_cmd_cli.py INVALID_MODE --dry-run

```

Pass criteria:

- valid commands normalize and publish;
- invalid values fail with a non-zero status;
- transition reasons are visible;
- STOP can be requested from every mode;
- denied transitions fail closed and explain why.

## Stage C4 — source arbitration and stale-command behavior

Keep drivetrain isolated. Start topic monitors:

```bash
ros2 topic echo /cmd_vel_mux
ros2 topic echo /cmd_vel

```

Publish a low bounded manual test input:

```bash
ros2 run savo_control mode_cmd_cli.py MANUAL
ros2 topic pub --rate 5 /cmd_vel_manual geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"

```

Confirm only the selected source reaches the mux/shaper. While still in MANUAL, publish a different value to `/cmd_vel_nav`; it must not take ownership.

Stop the manual publisher and observe the finite timeout. Output must return to zero without requiring a second command.

Repeat for approved sources/modes with conservative non-motion values.

Pass criteria:

- one source owns output at a time;
- inactive sources cannot leak through;
- selected output is bounded and shaped;
- publisher loss produces zero within the configured timeout;
- changing to STOP immediately suppresses non-zero output.

## Stage C5 — external stop

With a non-zero bounded command active and drivetrain isolated:

```bash
ros2 topic pub --once /savo_control/external_stop std_msgs/msg/Bool "{data: true}"

```

Observe mode, reason, and output. Clear the external stop only through the supported contract and verify motion does not resume automatically without new authorization.

Pass criteria:

- external stop suppresses command output promptly;
- state/reason shows the stop source;
- stale pre-stop commands are not replayed;
- reauthorization is explicit.

## Stage C6 — shaper limits

Use step inputs that remain below the physical-test maximum. Record `/cmd_vel_manual`, `/cmd_vel_mux`, and `/cmd_vel` to a bag or CSV.

Verify:

- velocity clamps;
- acceleration/deceleration limits;
- zero crossing behavior;
- independent linear and angular limiting;
- NaN/invalid input rejection;
- timeout-to-zero behavior.
Compare measured slopes with the active YAML. Any mismatch blocks motion.

## Stage C7 — recovery coordination without motion

Inject the supported stuck/recovery test signals or use the package test launch in a non-actuating configuration:

```bash
ros2 launch savo_control recovery_test.launch.py --show-args

```

Use only arguments reviewed from `--show-args`. Monitor:

```bash
ros2 topic echo /savo_control/recovery_active
ros2 topic echo /savo_control/recovery_status
ros2 topic echo /cmd_vel_recovery

```

Verify request admission, state transitions, timeout, cancellation, and return to STOP. Recovery must not start when safety or authority prerequisites are false.

## Stage C8 — wheels-raised integration

Only after C0–C7, base, perception, encoder direction, and emergency-stop checks pass:

1. raise all four wheels;
2. assign one operator to the emergency stop;
3. start full Core safe-idle bringup;
4. confirm `STOP` and zero `/cmd_vel_safe`;
5. authorize MANUAL through the normal mode path;
6. apply the lowest proven useful bounded command;
7. verify wheel direction and smooth ramp;
8. stop the publisher and verify watchdog stop;
9. request STOP and verify immediate zero;
10. trigger external stop and verify zero;
11. repeat one conservative angular command.
Abort on reversed wheels, oscillation, unexpected source ownership, stale motion, or failure to stop.

## Stage C9 — guarded floor test

Prerequisites:

- locked geometry;
- base, localization, perception, and supervisor gates passed;
- clear test lane;
- emergency-stop operator and observer present.
Perform short, low-speed tests:

- forward and reverse;
- left/right mecanum strafe;
- clockwise/counter-clockwise rotation;
- command release and STOP;
- blocked-front safety stop;
- one approved recovery sequence at reduced limits.
Measure commanded versus observed direction, stop distance, lateral drift, and mode transition latency.

## Abort criteria

Abort immediately if:

- control starts outside STOP;
- an inactive source reaches output;
- output persists after timeout/STOP;
- command signs are wrong;
- the robot accelerates beyond the active limits;
- recovery bypasses safety or authority;
- duplicate nodes or command publishers appear;
- emergency stop is ineffective;
- TF/localization becomes invalid during a floor test.

## Evidence

Retain:

- commit and host;
- active control YAML files;
- node/topic graph;
- mode/reason/status output;
- recorded input/mux/shaped/safe command traces;
- external-stop and timeout timings;
- wheel-raised video;
- floor-test measurements;
- pass/fail decision and operator signatures.

## Acceptance

`savo_control` is accepted for the tested scope only when all required stages pass, every command source remains within the authority model, STOP is fail-closed, stale commands decay to zero, and physical behavior matches the configured limits.
