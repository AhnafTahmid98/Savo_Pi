# Robot SAVO pre-real-test source completion report

**Date:** 2026-08-02
**Robot baseline:** `Savo_Pi_2026-08-02_19-00-22.zip` (three-way rebased using the common `00-06-38` ancestor)
**SavoMind baseline:** `SavoMind_current(5).zip`

## Executive result

The remaining source-level pre-real-test work has been three-way merged onto the latest `19-00-22` Robot SAVO tree and completed across the Robot SAVO ROS workspace and SavoMind. The repository now contains production deployment/network assets, safe persistent-state operations, expanded typed bridge coverage, the native speech-to-SavoMind round trip, physical-playback acknowledgement, richer read-only UI integration, safe diagnostics, complete test documentation, and a corrected pre-real-test readiness validator.

This environment does not contain ROS 2 Jazzy, `colcon`, `rosdep`, generated Robot SAVO interfaces, or the physical robot. Therefore, this report does **not** claim that the final ROS build, Pi deployment, or physical validation has passed.

**Conclusion:** `SOFTWARE PRE-REAL-TEST SOURCE CLOSURE COMPLETE — ROS BUILD AND PHYSICAL VALIDATION BLOCKED`

## Phase result

| Phase | Result | Evidence |
|---|---|---|
| RT-0 zero-byte inventory | Complete | Runtime-required empty files eliminated; only 22 intentional ROS/package markers remain in the Robot tree. |
| RT-1 legacy cleanup | Complete | Uncompiled duplicate UI/mapping scaffolds and obsolete empty tests/configs removed after reference checks. |
| RT-2 localization diagnostics | Complete | Empty legacy executables removed; diagnostics use the C++ localization health path and observer. |
| RT-3 deployment/network | Source complete | Netplan/Chrony renderers, systemd units, role install/update scripts, socket preparation, and edge runtime-group wiring implemented. |
| RT-4 persistent operations | Complete | Backup, restore, integrity, overwrite protection, disk, health, storage preflight, and log rotation implemented and tested. |
| RT-5 bridge command boundary | Complete for existing authorized interfaces | STOP, cancellation, bounded teleop, named-location navigation, mapping mission/control/query, Scan360 request, save/verification/review/release observation, and supervisor query are typed and fail closed. Operator approval and generic ROS access remain forbidden. |
| RT-6 speech round trip | Source complete | C++ client and Python SavoMind Unix-socket server implement protocol v2, TTS WAV return, correlation, peer credentials, limits, timeouts, and physical-playback acknowledgement. |
| RT-7 UI integration | Source complete for pre-test use | Active C++ UI consumes typed mapping/location plus live bringup, control, safety, navigation, speech, transcript, and response data without command authority. |
| RT-8 diagnostics | Complete for pre-test gate | Diagnostics are bounded, machine-readable, non-moving by default, and reject direct PCA9685/GPIO bypasses. Movement/head tests require explicit physical authorization and approved ROS paths. |
| RT-9 documentation | Complete | Network, time, audio, RealSense, architecture, dependency, and staged test documentation written; editable diagrams are valid XML. |
| RT-10 dependencies | Source complete, target validation blocked | Dependency matrix and role installers are present; `rosdep` is unavailable in this environment. |
| RT-11 readiness validator | Complete | Correct PASS/BLOCKED/FAIL logic; checks real launch files, AM-8 wiring, zero-byte policy, deployment assets, safety defaults, speech v2, bridge, UI, observer, and backup/restore. |

## Speech protocol v2

The Robot and SavoMind now share one bounded Unix-domain-socket contract:

```text
SAVOSPRQ request
  WAV + request/session identity
       ↓
SAVOSPRS response
  transcript + reply + PCM16 WAV + playback token
       ↓
Robot physical speaker playback
       ↓
SAVOSPAK acknowledgement
       ↓
SAVOSPAR acknowledgement result
```

Properties:

- strict protocol version 2 framing;
- bounded request and response sizes;
- 16 kHz, mono, PCM16 WAV validation;
- request/session correlation;
- `SO_PEERCRED` UID allowlisting;
- connect and I/O timeouts;
- stable public error codes without internal path/provider leakage;
- microphone gating through the existing speech runtime;
- playback token acknowledgement before pending navigation may dispatch;
- fake-server and real cross-repository protocol tests.

## Read-only and safety audit

No new component introduced any of the following:

| Capability | Result |
|---|---:|
| Raw or arbitrary ROS command execution | No |
| Generic service/action proxy | No |
| AM-8 operator approval from SavoMind | No |
| Geometry auto-lock | No |
| D435 voxel auto-enable | No |
| Supervisor bypass | No |
| Navigation-readiness bypass | No |
| Goal-admission bypass | No |
| Observer command authority | No |
| UI movement authority | No |
| Direct diagnostic PCA9685/GPIO movement bypass | No |

All production startup defaults remain `STOP`. Geometry remains honestly provisional. The D435 voxel profile remains explicitly hardware-gated.

## Validation performed in this environment

### Robot source validators

- full bringup validator: **PASS**;
- observer validator: **PASS**;
- pre-real-test validator: **BLOCKED**, with repository checks passing and only these environment blockers:
  - `rosdep` unavailable.

In the temporary merge checkout, `git diff --check` passed. An exported ZIP will naturally report Git metadata as unavailable until the patch is applied in the live checkout.

The validator also confirmed:

- all required files are nonempty;
- 22 intentional empty markers and zero deferred empty documents;
- no launch references removed/empty localization executables;
- all nonempty YAML parses;
- all nonempty XML parses;
- all Python parses;
- 40 shell scripts pass `bash -n`;
- rendered Netplan and systemd assets validate;
- backup/restore integration passes;
- observer remains read-only;
- startup remains STOP;
- geometry remains motion-blocking and provisional;
- D435 voxel validation defaults false;
- AM-8 quality approval, review gateway, and contract version 2 are present;
- diagnostic movement safety, UI read-only integration, bridge boundary, and speech protocol v2 checks pass.

### Focused Robot source contracts

```text
25 passed, 0 failed
```

Coverage included diagnostics safety, bridge production assets/command boundary, speech round-trip contracts, UI integration, and localization diagnostics closure.

### SavoMind regression

```text
362 passed, 0 failed
```

The suite includes speech socket v2, deployment contracts, live typed bridge behavior, provider behavior, playback acknowledgement, and error-sanitization tests.

### Cross-repository speech smoke test

A real standalone C++ client built from the Robot `savo_speech` transport exchanged audio with the actual Python SavoMind socket server, received transcript/reply/WAV/playback token, and completed playback acknowledgement:

```text
PASS cross-repo speech protocol v2
```

### Additional source checks

- strict standalone C++ syntax/warnings passed for the speech transport, round-trip worker, and UI live-state core;
- backup/restore integration passed, including integrity and overwrite refusal;
- systemd units rendered and passed `systemd-analyze verify`;
- Netplan templates rendered without applying network changes;
- no unexplained zero-byte production files remain in either source tree.

## Validation not performed here

The following require the user’s ROS 2 Jazzy development PC or Raspberry Pis:

- final `colcon build` of affected packages;
- generated-message/link/runtime validation;
- full `colcon test` regression;
- `rosdep check/install` on PC, core, edge, and observer;
- launch-generation checks through installed ROS package shares;
- native edge service/container UID/GID test;
- ALSA/ReSpeaker/speaker test;
- live DDS/network test;
- any physical robot test.

Use the new hardware-free regression entry point on the ROS development PC before copying to the Pis:

```bash
cd ~/Savo_Pi

deploy/common/run_pre_real_test_regression.sh --clean-affected
```

## Remaining physical and target-machine blockers

1. Apply both source patches in the live Git checkouts.
2. Run the complete ROS regression on the Jazzy development PC.
3. Resolve any target-specific `rosdep` packages.
4. Prepare `/var/lib/robot_savo` and `/run/savomind` ownership on the Pis.
5. Measure and lock AM-0B geometry; do not lock estimated values.
6. Perform core and edge safe-idle boot with control in STOP.
7. Validate DDS, time sync, TF, sensors, power, audio, UI, bridge, SavoMind, and observer.
8. Test motors with wheels raised and an operator at the emergency stop.
9. Validate safety-stop, stale-input, cancellation, and supervisor authorization.
10. Perform manual mapping, save, verification, AprilTag registration, and review.
11. Perform guarded autonomous mapping and approve a real AM-8 release.
12. Perform saved-map LiDAR-only navigation.
13. Validate Mac/PC RViz, desktop/mobile dashboard, firewall, and Wi-Fi bandwidth.
14. Validate D435 filtering separately before enabling the voxel profile.

## Final readiness statement

The source implementation is ready for the final ROS 2 build/test gate. It is **not yet authorized for physical movement**. Movement remains blocked by provisional geometry and the required staged hardware validation.
