# Base Test Plan

## Objective

Verify the production C++ `savo_base` path is the sole drivetrain executor and fails closed for zero, STOP, stale/invalid commands, publisher loss, board errors, and shutdown before any guarded floor use.

## Scope

Configuration parsing, PCA9685 ownership, mecanum mixing/sign/inversion, `/cmd_vel_safe`, clamp/breakaway behavior, watchdog, diagnostic state, hardware initialization/write failure, wheel-raised directions, and guarded floor behavior.

## Test ownership

Base maintainer owns BAS-001–004. Hardware stages require a motion-test lead, emergency-stop operator, and floor-test observer.

## Safety classification

BAS-001–004 are `STATIC`, `UNIT`, `PC`, or `TARGET-NON-HARDWARE` / `NO-MOTION`; BAS-005–008 are `HARDWARE-NON-ACTUATING` or `HARDWARE-ACTUATING` / `WHEELS-RAISED` then `GUARDED-FLOOR-MOTION`; BAS-009 is `FAULT-INJECTION`/`RECOVERY`.

## Preconditions

- Geometry is measured/reviewed/locked for motion stages.
- Wiring, power, E-stop and stands are approved; all wheels clear the floor for BAS-006–007.
- Core safe-idle reports control `STOP`; perception and base watchdog are fresh.
- Four encoder signs are observed by localization, though base does not own encoders.
- PCA9685 chip-wide initialization sharing with head has an approved single-owner/concurrency decision.

## Required hardware

Core Pi, PCA9685/Freenove motor interface, four motors/wheels, base battery/current observation, rated stands and physical emergency stop. Four encoders are required for correlated direction evidence.

## Required software / configuration

ROS 2 Jazzy; base board/driver/kinematics/topics/watchdog YAML; dry-run backend; supported `motor_direction_test.py` and odometry diagnostics. Raw board writes are not a routine test interface.

## Interfaces under test

Input `/cmd_vel_safe`, `/safety/stop`, `/safety/slowdown_factor`; base/watchdog/state/heartbeat/diagnostic outputs. No service/action or TF is owned.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| BAS-001 | T0 `STATIC` | Confirm production C++ driver, sole `/cmd_vel_safe` input, board bus/address/channels, current all-wheel inversions, configured `max_duty`, reversal quench, finite watchdog and zero-on-shutdown. Compare wheelbase/track/radius with locked geometry. |
| BAS-002 | T1 `UNIT`/`PC` | Build/tests pass for configuration, mecanum mix, wheel commands, clamp/scaling, dry-run, topics and watchdog logic. Test existence is not physical validation. |
| BAS-003 | T2 `TARGET-NON-HARDWARE` | Run dry-run backend with drivetrain power isolated: zero/startup/STOP, forward/reverse/strafe/yaw mixes, clamp, slowdown, reversal quench, publisher loss, shutdown and diagnostics match active config. |
| BAS-004 | T6 `FAULT-INJECTION` | In dry-run/source-supported injection, NaN/non-finite/invalid slowdown (where implemented), board init/write failure and stale commands produce zero/unhealthy state. Record unsupported injection as N/A, not PASS. |
| BAS-005 | T3 `HARDWARE-NON-ACTUATING` | Detect/configure PCA9685 with motor power isolated; confirm one owner, startup zero, board health, channel map, and no output after shutdown. |
| BAS-006 | T4 `HARDWARE-ACTUATING` | Wheels raised: use the supported `/cmd_vel_manual -> control -> perception -> /cmd_vel_safe` path and `motor_direction_test.py --allow-motion --wheels-raised` at its approved minimum. Verify each FL/FR/RL/RR sign and encoder sign one direction at a time. |
| BAS-007 | T4 `HARDWARE-ACTUATING` | Wheels raised: verify forward/reverse/left/right/yaw patterns, breakaway/minimum useful command, output clamp, slowdown/STOP, command release, publisher loss/watchdog, and shutdown zeroing. |
| BAS-008 | T5 `INTEGRATION` | After all earlier physical gates pass, guarded floor tests verify low-speed direction, mecanum strafe/yaw and measured response/stop behavior inside the approved envelope. No threshold is invented if the repository lacks one. |
| BAS-009 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Safely interrupt publisher, safety input, or driver process in wheels-raised setup; output becomes zero and restart remains STOP with no replay. Hardware failure injection is only performed if electrically safe and supported. |

## Pass criteria

Only the C++ driver executes motors; all wheel/encoder signs match; output is bounded; STOP/watchdog/publisher loss/shutdown zero every channel; failures are diagnostic and fail closed; guarded behavior matches the approved envelope.

## Blocked criteria

Provisional/inconsistent geometry, unresolved shared PCA9685 ownership, missing E-stop/stands/operators/power evidence, target dependency/build unavailable, or no approved stop-distance/breakaway criterion for acceptance.

## Failure criteria

Nonzero startup/shutdown/stale output, wrong wheel sign/channel, bypass input, watchdog miss, unclamped output, unsafe board recovery, or hardware initialization falsely healthy.

## Abort criteria

Immediately E-stop and remove drive power for uncommanded motion, inability to stop, stand movement, wrong wheel, motor stall/heat/smell, electrical fault, person entering zone, stale safety/localization, or unexpected command owner.

## Evidence to retain

Commit/host/config/geometry, board/channel/owner inspection, input/mux/safe/output/watchdog traces, current/temperature if available, each wheel/encoder direction result and video, stop/publisher-loss timings, floor measurements, operators/reviewer.

## Regression triggers

Base source; motor board/GPIO/PWM/channel/address; inversion/sign; radius/wheelbase/track; clamp/breakaway/watchdog/quench; `/cmd_vel_safe`; power/wiring; PCA ownership; controller/perception contract.

## Current validation status

Automated source tests and a historical physical baseline exist. Current-source geometry, shared PCA initialization, wheel polarity, breakaway, watchdog/STOP and floor response require physical regression.

## Related documentation

- [Base package](../packages/savo_base.md)
- [Control plan](control_test_plan.md)
- [Perception plan](perception_test_plan.md)
- [Abort criteria](failure_and_abort_criteria.md)
