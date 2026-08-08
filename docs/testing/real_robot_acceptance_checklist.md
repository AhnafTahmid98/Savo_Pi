# Robot Savo Real-Robot Acceptance Checklist

## Purpose

Final staged acceptance record for the current Robot Savo source, configuration, hardware, and production-release scope.

This checklist does not replace the detailed component plans. Each item must be supported by retained evidence. Marking an item complete without executing the referenced test is not acceptance.

## Acceptance identity

```text
Test date/timezone:
Robot hardware revision:
Core hostname:
Edge hostname:
Core commit:
Edge commit:
SavoMind version/image digest:
ROS distribution:
Core OS:
Edge OS:
Geometry profile/revision/digest:
Robot mode:
Bringup profile:
Active map ID/revision/release:
Location release:
Test location:
Lead operator:
Emergency-stop operator:
Reviewer:
```

## Decision labels

| Label | Meaning |
| --- | --- |
| PASS | Requirement executed successfully with evidence |
| BLOCKED | Prerequisite or authorization is missing; test was not executed |
| FAIL | Requirement was executed and did not meet criteria |
| N/A | Outside the explicitly approved acceptance scope, with written rationale |

Any safety-critical FAIL blocks motion acceptance. A BLOCKED item may permit only a narrower non-motion scope when that limitation is explicitly approved.

## Gate A — repository and release identity

- Exact Core and Edge commits are recorded.
- Working trees are clean.
- `git diff --check` passes on both targets.
- `validate_full_bringup.sh` passes.
- `validate_observer.sh` passes on the observer workstation.
- `validate_pre_real_test_readiness.sh` contains no FAIL result.
- All BLOCKED results are understood and resolved for the requested acceptance scope.
- Changelog and affected documentation match the source.
- No generated artifacts, credentials, runtime databases, maps, or bags are accidentally tracked.

Evidence/reference:

```text

```

## Gate B — role dependencies, builds, and tests

### Core

- Core dependency installation passes.
- All 14 Core packages are present.
- `build_core.sh --clean --test` passes.
- Zero test failures/errors are recorded.
- `--allow-missing` was not used.

### Edge

- Edge dependency installation passes.
- All 10 Edge packages are present.
- `build_edge.sh --clean --test` passes.
- Zero test failures/errors are recorded.

### Observer and companion

- Observer build/validation passes.
- SavoMind protocol/version compatibility is recorded.
- Required container/model/provider health checks pass without exposing secrets.

Evidence/reference:

```text

```

## Gate C — physical inspection and power

With power removed where appropriate:

- Chassis, decks, wheels, mecanum rollers, fasteners, and mounts are secure.
- No cable can contact wheels, rollers, LiDAR, or head mechanism.
- LiDAR rotates freely.
- Head pan/tilt range is physically unobstructed.
- RealSense, Pi Camera, ToF, ultrasonic, and IMU mounts are secure.
- Battery and UPS modules show no swelling, heat damage, loose connector, or exposed conductor.
- Fuse/protection and power-distribution arrangement match the approved wiring record.
- Emergency-stop mechanism is accessible and understood by both operators.
- The test stand can safely raise all four wheels.
- The floor test area has a controlled boundary and clear abort path.

Power-on checks:

- Core and Edge boot reliably.
- Supply voltage remains within approved limits at idle.
- UPS state is visible and plausible.
- No component overheats or repeatedly resets.
- Controlled shutdown request and physical power-down behavior pass.

Evidence/reference:

```text

```

## Gate D — geometry, URDF, and TF

- Geometry was physically measured.
- Measurement metadata and reviewer are recorded.
- Profile state is `locked` for motion acceptance.
- `validate_geometry_profile.py --require-locked` passes.
- Canonical geometry digest is recorded.
- Xacro generates successfully.
- `check_urdf` passes.
- Nav2 footprint encloses the physical robot and fixed protrusions.
- `base_footprint -> base_link` matches the measured robot.
- Wheel and sensor frames are in the correct quadrants/orientations.
- Description does not publish dynamic localization transforms.
- Head dynamic TF has one authority.
- Live sensor frame IDs match the description.
- TF tree is connected and contains no duplicate authorities.

Evidence/reference:

```text

```

## Gate E — network, clock, storage, and services

- Dedicated Core–Edge link addresses match the approved configuration.
- ROS domain and middleware match Core, Edge, and observer.
- Core and Edge discover each other after boot.
- Time synchronization is stable and within the approved offset.
- Persistent Core state/log directories exist and are writable by the service user.
- `/run/savomind` ownership/mode/group are correct.
- Bridge runtime directory/socket ownership is correct for the selected service model.
- Only one service owns each role and component.
- Rendered systemd units pass verification.
- Core and Edge services stop cleanly.
- Restart-on-failure behavior is bounded and does not loop uncontrollably.
- Safe reboot behavior passes.
- Current state backup and known-good rollback are available.

Evidence/reference:

```text

```

## Gate F — safe-idle distributed bringup

Start Core first, then Edge.

- Core reports `safe_idle`.
- Core control startup mode is `STOP`.
- No wheel movement occurs at startup.
- Supervisor starts in the expected non-armed state.
- Core bringup state/heartbeat are current.
- Edge reports `safe_idle` with the approved profile.
- RealSense and VO have one owner each.
- Bridge has one owner.
- Speech and UI match their approved feature flags.
- D435 obstacle cloud is disabled unless its separate gate passed.
- Edge bringup state/heartbeat are current.
- Observer displays current state without creating control authority.

Evidence/reference:

```text

```

## Gate G — base, motors, and encoders

Complete first with all wheels raised.

- Base starts with zero output.
- Command watchdog stops output after publisher loss.
- STOP produces zero drivetrain output.
- Physical emergency stop removes or blocks unsafe motion as designed.
- Front-left wheel direction is correct.
- Front-right wheel direction is correct.
- Rear-left wheel direction is correct.
- Rear-right wheel direction is correct.
- Forward command produces the approved mecanum wheel pattern.
- Reverse command produces the approved pattern.
- Left/right strafe patterns are correct.
- Clockwise/counter-clockwise patterns are correct.
- Minimum useful duty/speed is recorded.
- Maximum test duty/speed remains within the approved envelope.
- Encoder count direction matches wheel direction for all wheels.
- Encoder stale/disconnect behavior is visible.
- No motor channel remains energized after STOP/shutdown.

Evidence/reference:

```text

```

## Gate H — control and command authority

- Startup mode is STOP.
- Valid mode transitions work.
- Invalid transitions are rejected with a reason.
- Only the selected source reaches `/cmd_vel`.
- Inactive manual/auto/nav/recovery sources cannot leak through.
- Command shaping matches configured velocity/acceleration limits.
- Publisher timeout returns output to zero.
- External stop suppresses output.
- Pre-stop commands do not resume automatically.
- Supervisor authority revoke interrupts motion ownership.
- Recovery commands use `/cmd_vel_recovery` and remain gated.
- No Edge, UI, observer, bridge, or SavoMind component publishes directly to `/cmd_vel_safe` or motor hardware.

Evidence/reference:

```text

```

## Gate I — near-field safety and perception

- Left ToF reports plausible range and correct side identity.
- Right ToF reports plausible range and correct side identity.
- Front ultrasonic reports plausible range.
- Sensor timestamps remain current.
- Missing/stale required sensor input fails closed.
- Front slowdown threshold is measured and recorded.
- Front stop threshold is measured and recorded.
- Side slowdown/stop behavior is verified for mecanum motion.
- Safety-stop state is visible to control/supervisor/UI.
- Non-zero `/cmd_vel` is suppressed or scaled as configured.
- `/cmd_vel_safe` returns to zero on stale command input.
- Obstacle removal does not cause uncontrolled automatic resume.
- False positives/negatives in the approved environment are documented.

Evidence/reference:

```text

```

## Gate J — LiDAR and localization

### LiDAR

- RPLIDAR serial ownership is unique.
- `/scan` frame is `laser_frame`.
- Scan rate and angular coverage meet the approved baseline.
- Filtering does not remove required obstacles.
- Motor/scan dropout is detected.
- Restart/recovery is bounded.

### IMU and wheel odometry

- IMU orientation and yaw sign are correct.
- IMU covariance and stale behavior are acceptable.
- Wheel odometry forward/reverse signs are correct.
- Strafe and yaw signs are correct.
- Distance and rotation scale are measured.

### EKF

- `odom -> base_footprint` has one authority.
- `/odometry/filtered` is current.
- Stationary drift is measured.
- Straight, strafe, and rotation tracks are plausible.
- Sensor dropout behavior is controlled.
- VO remains excluded until its fusion gate is approved.

Evidence/reference:

```text

```

## Gate K — Edge perception, VO, audio, UI, and bridge

### RealSense and VO

- D435 is connected over the approved USB path.
- Serial/firmware/stream profile are recorded.
- Color and depth rates are stable.
- Frame IDs and timestamps are correct.
- Device loss and restart behavior are bounded.
- VO output is stable with plausible covariance.
- VO dropout and recovery are visible.
- VO fusion into EKF is enabled only after guarded validation.

### D435 obstacle cloud, when in scope

- Self-filter bounds are measured from the real robot.
- Robot body and floor artifacts are filtered correctly.
- Cloud frame and timing are correct.
- LiDAR remains the approved clearing source.
- Dropout fails safely.
- Nav2 voxel profile is enabled only after all cloud tests pass.

### Speech

- ReSpeaker capture device is stable.
- Speaker playback device is stable.
- Wake word and VAD operate within the tested environment.
- Utterance segmentation is bounded.
- Protocol-v2 request/session correlation passes.
- Returned WAV is validated before playback.
- Physical playback acknowledgement is correct.
- Cancellation and microphone gating prevent feedback loops.
- SavoMind failure/timeouts remain bounded.

### UI

- 800×480 framebuffer output is correct.
- Touch input and calibration are correct when enabled.
- Safety/control/navigation/mapping/speech/power states are current.
- Stale data is visually distinguishable.
- UI remains read-only.

### Bridge

- Snapshot and command socket ownership are correct.
- Intended peer is accepted.
- Unauthorized peer is rejected.
- Typed STOP works fail-closed.
- Teleoperation is bounded.
- Navigation/mapping requests require current readiness/authority.
- Timeout and stale state are rejected.
- No generic ROS forwarding or shell execution exists.
- Operator-only approval is not exposed.

Evidence/reference:

```text

```

## Gate L — supervisor and power authority

- Supervisor persistent state loads successfully.
- Startup remains non-armed.
- Core/Edge readiness requirements are visible.
- Arming requires the approved prerequisites.
- Faults latch as designed.
- Fault clear/recovery requires explicit action.
- Authority revoke stops or cancels active motion/mission.
- Map context mismatch blocks mission authority.
- Controlled shutdown request is issued under the approved low-power condition.
- Invalid battery percentage does not create false confidence.
- Power-state loss or stale input is reported.
- Restart preserves required supervisor state without replaying unsafe commands.

Evidence/reference:

```text

```

## Gate M — first guarded floor motion

Prerequisites: Gates A–L applicable to motion are PASS.

- Clear low-risk lane established.
- Emergency-stop operator positioned.
- Maximum test speed/acceleration reduced and recorded.
- Short forward motion passes.
- Stop-on-command passes.
- Stop-on-publisher-loss passes.
- Short reverse motion passes.
- Left/right strafe passes.
- Clockwise/counter-clockwise rotation passes.
- Endpoint drift and heading error are measured.
- Front obstacle slowdown/stop passes.
- Side obstacle behavior passes.
- External stop and authority revoke pass during motion.
- No unexpected vibration, wheel slip, cable movement, or overheating occurs.

Evidence/reference:

```text

```

## Gate N — mapping and production release

- Manual mapping launch starts with STOP.
- Live SLAM TF ownership is correct.
- Small-area manual map is coherent.
- Loop closure is acceptable.
- Map session save produces all required artifacts.
- Saved-map verification passes.
- Map quality evaluation passes.
- Semantic candidates remain unapproved until operator review.
- Operator review evidence is retained.
- Production release create/verify/promote passes atomically.
- Active-map contract contains correct map/revision/release/geometry identity.
- Restart verifies the same immutable release.
- Corrupt copied release is rejected.
- Previous release/backup remains recoverable.

Autonomous mapping, when in scope:

- Mission authority is required.
- Action admission, feedback, cancel, and timeout pass.
- Frontier completion is stable.
- Coverage requires explicit approval.
- Recovery is bounded.
- Auto-save uses the package-owned save path.
- Quality/review/release remain separate gates.

Evidence/reference:

```text

```

## Gate O — navigation and named locations

- Production navigation verifies the active AM-8 release.
- Missing/hash-mismatched/copied invalid release is rejected.
- AMCL and map server lifecycle are correct.
- Initial pose converges.
- Nav readiness matches active release identity.
- Goal without authority is rejected.
- Goal with stale readiness is rejected.
- Short wheels-raised goal command path is correct.
- First short floor goal passes.
- Mid-route cancel stops within the approved bound.
- Obstacle stop/replan passes.
- Recovery is bounded and interruptible.
- Mecanum lateral/diagonal tracking is measured.
- Goal position/yaw tolerances are met.
- Named location matches active map/revision/release.
- Mismatched location release is rejected.
- Restart and repeat short production goal pass.

Evidence/reference:

```text

```

## Gate P — recovery, backup, and incident readiness

- Service failure evidence is retained before restart.
- Core role restart returns to STOP.
- Edge role restart does not create duplicate owners.
- Staged update failure leaves the active install unchanged.
- Previous install rollback is demonstrated.
- State backup integrity verifies.
- Restore test passes in a controlled destination.
- Overwrite protection works.
- Corrupt archive/path traversal/symlink payload is rejected.
- Map, location, supervisor, and release identities remain consistent after restore.
- Operators know emergency shutdown and log collection procedures.

Evidence/reference:

```text

```

## Deviations and limitations

Record every skipped, modified, or limited test:

```text
Item:
Reason:
Risk:
Compensating control:
Owner:
Expiry/retest date:
```

## Final acceptance decision

| Scope | PASS | BLOCKED | FAIL | N/A |
| --- | --- | --- | --- | --- |
| Source and target builds | [ ] | [ ] | [ ] | [ ] |
| Safe-idle distributed system | [ ] | [ ] | [ ] | [ ] |
| Wheels-raised motion | [ ] | [ ] | [ ] | [ ] |
| Guarded manual floor motion | [ ] | [ ] | [ ] | [ ] |
| Manual mapping and release | [ ] | [ ] | [ ] | [ ] |
| Autonomous mapping | [ ] | [ ] | [ ] | [ ] |
| Production navigation | [ ] | [ ] | [ ] | [ ] |
| Speech/UI/SavoMind integration | [ ] | [ ] | [ ] | [ ] |
| D435 voxel navigation | [ ] | [ ] | [ ] | [ ] |
| Recovery and rollback | [ ] | [ ] | [ ] | [ ] |

Approved operating envelope:

```text
Environment:
Maximum linear speed:
Maximum angular speed:
Allowed robot modes:
Allowed bringup profiles:
Approved map/location releases:
Required operators:
Known limitations:
Retest triggers:
```

Sign-off:

```text
Lead developer:
Test operator:
Emergency-stop operator:
Safety reviewer:
Project owner:
Decision date/time:
Overall decision: ACCEPTED / CONDITIONALLY ACCEPTED / BLOCKED / REJECTED
```
