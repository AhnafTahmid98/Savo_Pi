# Safe-idle commissioning checklist

## Purpose and gate

This is the final Phase 6 gate for a fresh Core, Edge, observer, and Robot Savo-side SavoMind installation. It verifies detection, access, builds, distributed startup, and fail-closed behavior without commanding motion.

It does not certify production operation. Mark each check with evidence and finish with `PASS`, `BLOCKED`, or `FAIL`.

## Installation identity

```text
Date:
Installer/reviewer:
Core commit:
Edge commit:
Observer commit:
SavoMind version:
Core hostname:
Edge hostname:
ROS distro:
ROS domain:
RMW implementation:
Geometry state:
Evidence location:
```

## Safety prerequisites

- [ ] Emergency stop is present, accessible, and covered by its formal test status.
- [ ] Robot is secured against unintended motion.
- [ ] Core control startup mode is `STOP` and robot mode is `safe_idle`.
- [ ] Locked geometry is required; provisional geometry and unvalidated D435 voxel integration remain blocked.
- [ ] No step below publishes velocity, changes control mode, starts navigation, or starts mapping.

## Core

- [ ] Ubuntu 24.04 ARM64, ROS 2 Jazzy, and hostname `core`/`savo-core` verified.
- [ ] Core dependency installer passed at the recorded revision.
- [ ] All 14 Core packages built/tested with zero failures/errors.
- [ ] `/var/lib/robot_savo` and `/var/log/robot_savo` have the selected runtime identity and `0750` mode.
- [ ] Exactly one Core role owner is selected; generic and role-specific units are not duplicated.
- [ ] Interactive Core safe idle starts with supervisor/readiness visible and `STOP` preserved.
- [ ] Basic non-actuating serial, I2C, and GPIO access is recorded.

## Edge

- [ ] Ubuntu 24.04 ARM64, ROS 2 Jazzy, and hostname `edge`/`savo-edge` verified.
- [ ] Edge dependency installer passed at the recorded revision.
- [ ] All 10 Edge packages built/tested with zero failures/errors.
- [ ] `/run/savomind` group/mode and reboot recreation are verified.
- [ ] `/run/savo_bridge` is created by the selected standalone owner or explicitly provisioned for distributed ownership.
- [ ] Exactly one Edge, bridge, UI, RealSense, and VO owner is selected.
- [ ] RealSense is detected at USB 3 speed; configured serial and required streams/status are recorded.
- [ ] VO starts and reports current status; performance validation is deferred.
- [ ] ReSpeaker is detected and stable `savo_respeaker` capture/playback resolves.
- [ ] UI framebuffer/touch hardware is detected if UI will later be enabled.
- [ ] Edge UPS is detected and returns a finite read.
- [ ] Interactive Edge safe idle starts with speech/UI/obstacle cloud still disabled unless their separate gates passed.

## Network, time, and observer

- [ ] Actual interfaces, addresses, routes, and recovery path are recorded.
- [ ] Dedicated link ping succeeds in both directions.
- [ ] `chronyc tracking`/`sources -v` show acceptable agreement and intended source selection.
- [ ] Core, Edge, and observer have the same reviewed ROS domain and compatible RMW.
- [ ] Each host sees the expected distributed ROS graph without duplicate owners.
- [ ] Observer build and source validator pass.
- [ ] Observer connection check, RViz/dashboard observation, and read-only authority are verified.

## Hardware visibility

Record detection/access/basic data only; formal accuracy, dynamics, timing, safety response, and motion tests remain deferred.

- [ ] LiDAR serial device and non-actuating stream access.
- [ ] IMU I2C detection/basic data.
- [ ] Encoder GPIO access without wheel movement.
- [ ] ToF mux/sensors I2C detection/basic data.
- [ ] Ultrasonic GPIO access/basic passive readings.
- [ ] Head I2C/GPIO access without servo commands.
- [ ] Core camera detection if fitted.
- [ ] Edge RealSense color/depth stream detection.
- [ ] Audio capture and controlled low-volume playback.
- [ ] Core/Edge UPS and Core base-battery ADC visibility.
- [ ] Display/touch visibility if enabled later.

## Distributed safe idle

- [ ] Start Core interactively; confirm role, `safe_idle`, `lidar_only`, and `STOP`.
- [ ] Start Edge interactively; confirm current default feature flags and one bridge owner.
- [ ] Confirm no duplicate node/component owners.
- [ ] Confirm no unexpected `/cmd_vel` publisher or motor output.
- [ ] Confirm supervisor/readiness, diagnostics, TF, camera/VO, and power state are current as applicable.
- [ ] Confirm no physical motion occurred.
- [ ] Stop both roles cleanly and inspect the journals before service enable/start.

## Validators

```bash
cd "$HOME/Savo_Pi"
bash deploy/common/validate_full_bringup.sh
bash deploy/observer/validate_observer.sh
bash deploy/common/validate_pre_real_test_readiness.sh
git diff --check
```

Record each result exactly. A readiness `BLOCKED` caused by target hardware, geometry measurement, or an unavailable host facility remains a blocker; do not alter configuration to suppress it.

## Gate result

```text
Result: PASS | BLOCKED | FAIL
Reviewer:
Date/time:
Blocking/failing item(s):
Evidence:
Approved next step:
```

- `PASS`: Installation and safe-idle commissioning complete. Ready to begin formal validation.
- `BLOCKED`: Setup reached a safe stopping point but required external hardware, host policy, measurement, or configuration evidence is missing.
- `FAIL`: A required contract was violated or an unsafe/unexpected behavior occurred.

`PASS` does not mean “robot approved for autonomous operation.” Motion requires the [component validation overview](../testing/component_validation_overview.md), [full robot test plan](../testing/full_robot_test_plan.md), and [real-robot acceptance checklist](../testing/real_robot_acceptance_checklist.md).

## Abort and evidence handling

On movement, non-STOP state, unexpected command source, stale safety state, duplicate authority, electrical concern, or device fault: use the physical emergency stop if needed, stop services, preserve evidence, and follow [emergency recovery](../operations/emergency_stop_and_recovery.md). Do not retry by weakening a safety gate.
