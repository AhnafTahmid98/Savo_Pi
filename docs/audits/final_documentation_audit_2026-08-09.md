# Robot Savo Final Documentation Audit

## Audit scope

Repository-wide Phase 7 audit of current production packages, interfaces, launch/configuration, deployment roles/services, hardware owners, storage, operations, test coverage, requirements, regression and evidence governance. The audit was executed on 2026-08-09 and did not start robot nodes, actuate hardware, publish motion, apply network configuration, or modify production state.

## Repository identity

| Field | Value |
| --- | --- |
| Repository | `~/Savo_Pi` |
| Inspected commit | `77098cf38d299f572152f0b7b806ea4b5afeaf16` |
| Working tree | Documentation changes present, including pre-existing Phase 6 edits; source behavior unchanged by Phase 7 |
| Target platform | Ubuntu 24.04 / ROS 2 Jazzy; Core and Edge Raspberry Pi 5 |
| Execution environment | Managed development sandbox, not a target Pi or hardware session |

## Methodology

The audit used current source/interfaces first, followed by launch/config, manifests/install rules, role arrays/deployment/systemd, validators/tests and then documentation. It inventoried manifests/pages/tests/plans/interfaces/TF/hardware/storage/units/commands/links/placeholders/stale names, inspected the complete test and package document sets, ran non-actuating validators and attempted the repository hardware-free regression.

Static searches included ROS publishers/subscriptions/services/actions and parameter/remapping sources; literal-only search was not treated as sufficient where interfaces are parameterized. No live ROS graph was available, so runtime ownership remains a target/integration verification task.

## Package coverage

Exactly 20 manifests and 20 corresponding `docs/packages/savo_*.md` pages exist. `package_ownership_matrix.md` is not counted as a package page. `savo_msgs` is an interface package and is verified by interface generation/build/inventory/consumer compatibility rather than a physical plan.

Source contains 435 Python/C++ test files under package `test/`/`tests/` trees; every package has automated test source. “Present” below does not mean the entire package test suite passed in this execution.

| Package | Automated tests | Central test plan | Target test | Hardware test | Integration test |
| --- | --- | --- | --- | --- | --- |
| `savo_base` | Present (7) | `BAS` | NOT RUN | Historical baseline; current regression required | NOT RUN |
| `savo_bringup` | Present (5) | `BRG` | NOT RUN | N/A | NOT RUN |
| `savo_bridge` | Present (20) | `BRD` | BLOCKED by sandbox sockets/log path | N/A | NOT RUN |
| `savo_control` | Present (49) | `CTL` | NOT RUN | Historical baseline; current regression required | NOT RUN |
| `savo_description` | Present (8) | `DSC` | NOT RUN | Measurement/geometry lock BLOCKED | NOT RUN |
| `savo_head` | Present (22) | `HED` | NOT RUN | Historical baseline; current regression required | NOT RUN |
| `savo_lidar` | Present (14) | `LID` | NOT RUN | Historical baseline; current regression required | NOT RUN |
| `savo_localization` | Present (19) | `LOC` | BLOCKED by sandbox DDS runtime | Historical baseline; current regression required | NOT RUN |
| `savo_locations` | Present (24) | `LCT` | NOT RUN | N/A | NOT RUN |
| `savo_mapping` | Present (79) | `MAP` | BLOCKED by sandbox DDS runtime/timeouts | NOT RUN | NOT RUN |
| `savo_msgs` | Present (11) | Interface/traceability coverage | NOT RUN | N/A | NOT RUN |
| `savo_nav` | Present (47) | `NAV` | BLOCKED by sandbox DDS/XML runtime | NOT RUN | NOT RUN |
| `savo_observer` | Present (8) | `OBS` | Source validator PASS; workstation build NOT RUN | N/A | NOT RUN |
| `savo_perception` | Present (4) | `PER` | NOT RUN | Historical Core baseline; D435 cloud BLOCKED | NOT RUN |
| `savo_power` | Present (23) | `PWR` | NOT RUN | NOT RUN | NOT RUN |
| `savo_realsense` | Present (17) | `RLS` | NOT RUN | NOT RUN | NOT RUN |
| `savo_speech` | Present (31) | `SPH` | BLOCKED by sandbox Unix sockets/XML access | NOT RUN | NOT RUN |
| `savo_supervisor` | Present (27) | `SUP` | NOT RUN | N/A | NOT RUN |
| `savo_ui` | Present (4) | `UI` | Affected PC tests observed PASS before aggregate stop; target NOT RUN | NOT RUN | NOT RUN |
| `savo_vo` | Present (16) | `VO` | NOT RUN | NOT RUN | NOT RUN |

The required core set now contains 19 subsystem test plans plus the acceptance checklist. `savo_msgs` appears in interface inventory and traceability. No current test plan requires `savo_intent` or `savo_dashboard` as a production package.

## Architecture coverage

All 18 architecture pages were reconciled as the current architecture set. Authority boundaries remain consistent: base is final motor owner; perception is the independent command gate; supervisor permits but does not execute; bridge is typed; UI/observer are read-only; mapping quality/review/release and location operator approval remain separate.

No documentation correction weakened STOP, geometry, release, D435 or operator-approval gates.

## Hardware coverage

All 11 hardware pages cover the configured BOM, wiring, GPIO/I2C, mounting, geometry/calibration and power ownership. Configured values remain explicitly provisional where not measured. No unowned production device was found in the source/document comparison.

Known shared-device finding remains: base and head independently initialize the same Core PCA9685 at bus 1 address `0x40`. Channels do not overlap, but chip-wide frequency/reset/concurrent access is unresolved and blocks equivalent current hardware acceptance.

## Deployment coverage

Eight deployment pages cover role builds, Core/Edge deployment, startup, systemd, release, recovery and companion Docker boundary. Role arrays contain 14 Core and 10 Edge packages; observer is workstation-only. The validators confirmed package/default/storage/geometry/D435/approval contracts.

The service inventory was corrected to include `savo-ui-runtime.service`. Its dependency/ownership relationship with the Edge role remains an implementation/deployment finding.

## Setup coverage

The 15 setup pages cover development, Core, Edge, observer, SavoMind, ROS/domain/network/time, permissions, environment/secrets, RealSense, audio, UPS and commissioning. Setup does not claim motion authorization. Existing hard-bound D435 serial, missing repository-supplied ALSA alias, general udev policy and distributed bridge runtime-directory requirements remain explicit commissioning items.

## Operations coverage

The 16 operations pages cover operator startup/inspection/shutdown, emergency response, manual drive, mapping, navigation, speech/UI, map/location administration, troubleshooting, logs, maintenance, backup/restore and incidents. Stale component sheets were rewritten so they no longer recommend direct PCA motor writes, ungated `/cmd_vel`, legacy `/e_stop`, or a removed VO launch.

## Testing coverage

All required component plans exist with stable IDs, classification/risk, prerequisites, interfaces, staged static/PC/target/hardware/integration/fault/recovery coverage, PASS/BLOCKED/FAIL/abort/evidence/regression/status sections as applicable. Existing detailed control/description procedures were retained and assigned stable ID mappings. Mapping, navigation, localization, perception, VO and speech were source-reconciled rather than left skeletal.

The full-robot plan is dependency-ordered from source/build/commissioning/safe-idle through non-actuating hardware, wheels-raised, guarded floor, localization, mapping/release, navigation, Edge integration, recovery and acceptance. The acceptance checklist references component/system IDs and requires explicit return-to-service authorization.

## Requirements traceability

[The traceability matrix](../testing/requirements_traceability_matrix.md) contains:

| Metric | Result |
| --- | ---: |
| Total requirements | 54 |
| Implementation traced | 54 |
| Verification traced | 54 |
| Current source/historical evidence | 15 |
| `BLOCKED` | 22 |
| `NOT_RUN` | 17 |
| Missing implementation owner | 0 |
| Missing verification method | 0 |

The evidence count is 10 `SOURCE_PASS` plus 5 `HISTORICAL_BASELINE`; historical evidence is not current hardware PASS.

## Regression coverage

[The regression matrix](../testing/regression_matrix.md) defines R0–R5 and minimum retest scopes for package source, motion hardware/config, all sensors, geometry/TF/footprint, D435/VO, mapping/location/navigation releases, supervisor/bridge/speech/UI/power, network/time/systemd/deployment/ROS/storage and `savo_msgs` changes.

It explicitly couples geometry digest, sensor TF, footprint, D435 self-filter, map release and location release. Material geometry changes require compatibility evidence or regeneration/remapping/re-release; old production maps are not presumed valid.

## ROS interface audit

The `savo_msgs` inventory is complete: 10 messages, 22 services and 6 actions are represented in the central package page and owning subsystem/traceability documentation. Source declarations and launch/config parameters were compared with Phase 3/4 pages.

| Audit result | Count |
| --- | ---: |
| Undocumented public topics | 0 unexplained |
| Undocumented public services | 0 unexplained |
| Undocumented public actions | 0 unexplained |
| Stale documented interfaces presented as current | 0 unexplained |
| Message-type/producer/consumer mismatch | 0 unexplained |

Three legacy source references remain explicitly documented rather than presented as current interfaces: perception consumer metadata names retired `savo_dashboard`; a legacy Python UI subscriber names `/savo_intent/intent_result`; a retained Python base docstring names `savo_intent`. None is in role arrays or the production C++ motion path.

## TF authority audit

Documented/source ownership is consistent:

- SLAM and AMCL own `map -> odom` in mutually exclusive modes;
- localization EKF owns `odom -> base_footprint`; wheel odometry TF is disabled;
- description owns fixed robot/sensor TF and RealSense driver TF is disabled;
- head owns its dynamic chain but current publication/calibration is disabled pending validation;
- VO publishes odometry data, not authoritative Core TF.

| Result | Count/status |
| --- | --- |
| Duplicate documented owners | 0 unexplained |
| Source transforms missing from docs | 0 unexplained |
| Documented transforms absent from source contract | 0 unexplained |
| Mode-dependent conflicts | 0 when launch exclusivity holds |
| Live runtime TF validation | NOT RUN |

## Hardware ownership audit

| Hardware | Owner | Host | Interface / identity | Validation state |
| --- | --- | --- | --- | --- |
| Motors/PCA9685 | `savo_base` plus head shared-chip interaction | Core | I2C-1 `0x40`, motor channels 0–7 | Historical baseline; shared init BLOCKED |
| Encoders | `savo_localization` | Core | Four configured GPIO pairs | Historical baseline/current regression |
| RPLIDAR A1 | `savo_lidar` | Core | `/dev/ttyUSB0`, 115200 | Non-persistent identity; current regression |
| BNO055 | `savo_localization` | Core | I2C-1 `0x28` | Current orientation/drift regression |
| TCA9548A/ToFs | `savo_perception` | Core | `0x70`, channels 2/3, sensors `0x29` | Identity/threshold regression |
| Ultrasonic | `savo_perception` | Core | GPIO 27/22 | Level-shift/threshold regression |
| Pi Camera/head servos | `savo_head` | Core | libcamera; PCA channels 15/14 | Mount/limits/shared-chip regression |
| RealSense D435 | `savo_realsense` | Edge | USB3; configured serial `801212070967` | Live identity/profile NOT RUN |
| ReSpeaker/speaker | `savo_speech` | Edge | ALSA devices | Alias/device/acoustic BLOCKED |
| UPS monitors | `savo_power` | Core/Edge | independent I2C-1 `0x36` | Calibration/fault NOT RUN |
| ADS7830/base battery | `savo_power` | Core | I2C `0x48`, channel 2 | Calibration NOT RUN |
| Display/touch | `savo_ui` | Edge | framebuffer/input, 800×480 | Target NOT RUN |

No undocumented/unowned hardware was found. The shared PCA chip is a duplicate-access risk, not an unexplained owner.

## Persistent-state audit

Current production paths under `/var/lib/robot_savo`, `/var/log/robot_savo`, `/run/savo_bridge`, `/run/savomind`, workspace `.releases` and `install.previous.*` are documented with owners/purpose/backup or rollback policy. No production persistent writer defaulting to `/tmp` was found; UI preview/VO debug/test paths are non-production/disabled/ephemeral.

The aggregate validator passed isolated backup/restore integrity, overwrite refusal and path safety. No production database was modified. Distributed bridge bringup still depends on explicit `/run/savo_bridge` provisioning when standalone systemd does not own it.

## Service/systemd audit

Nine service units/templates exist across deploy/package trees:

`savo.service`, `savo_core.service`, `savo_edge.service`, `savo_mapping.service`, `savo-location-stack@.service`, `savo-ui-runtime.service`, `savo-ui.service`, `savo_bridge.service.in`, and `savo-supervisor.service.in`.

All are now represented in deployment documentation. Render/`systemd-analyze verify` passed without installation. The Edge unit’s `Wants=/After=savo-ui-runtime.service` despite `SAVO_START_UI=false`, plus the setup decision not to install that companion automatically, remains unresolved and must be reconciled to one UI owner.

## Command-existence audit

Custom `ros2 launch`/`ros2 run` references across README/docs were compared with package manifests, launch files, CMake installed targets/programs and Python entry points. Standard external ROS tools were classified separately.

| Result | Count |
| --- | ---: |
| Unresolved Robot Savo launch commands | 0 |
| Unresolved Robot Savo run commands | 0 |
| Invented package commands | 0 unexplained |
| Stale commands corrected | 2 (`savo_vo.launch.py`; legacy RPLIDAR package launch) |
| Unsafe stale procedures corrected | 3 (direct PWM, ungated velocity, legacy `/e_stop`) |

## Link audit

A relative Markdown link audit over `README.md`, `CHANGELOG.md` and `docs/**/*.md` found **0 broken relative links** after Phase 7. No empty Markdown file exists. Required testing/governance/audit documents are indexed.

## Placeholder/stale-reference audit

No unexplained TODO/TBD/scaffolding marker remains in current Markdown. “Placeholders” in systemd rendering prose and the dated zero-byte audit are intentional descriptive/historical terms. Remaining `savo_intent`, `savo_dashboard`, and `savo_uwb` documentation occurrences explicitly classify them as retired/nonexistent/future; none is presented as a current package.

## Validation results

| Validation | Result | Scope / detail |
| --- | --- | --- |
| `validate_full_bringup.sh` | PASS | Required assets, parsers, role arrays, paths and source contracts |
| `validate_observer.sh` | PASS | Assets/parsers and read-only source/RViz enforcement |
| `validate_pre_real_test_readiness.sh` | PASS | Required files, whitespace, parsers, shell, systemd, backup/restore, dependencies and safety/authority defaults; Netplan schema detail BLOCKED by no system D-Bus but aggregate had no FAIL |
| `run_pre_real_test_regression.sh --help` | PASS | Confirmed hardware-free, never launches nodes or publishes commands |
| Hardware-free affected-package regression | BLOCKED | Build began and many static/unit tests passed, but managed sandbox denied ROS log writes, local Unix socket bind/listen, DDS UDP/getifaddrs and remote XML schema access; mapping runtime timeouts made continued execution invalid, so run was stopped (exit 130) |
| Package/link/command/static audits | PASS | Counts and zero unexplained mismatches above |
| `git diff --check` | PASS | Final whitespace check |

The affected regression’s test XML records failures caused by the denied environment, including bridge/speech sockets/log paths, mapping/nav DDS discovery and XML schema fetch. These are not converted to source PASS and are not silently classified as production defects; rerun in an unrestricted ROS test environment is required.

## Historical hardware baseline

Earlier physical-robot evidence exists for drivetrain, Core sensors/localization/perception and other subsystems. It remains `HISTORICAL_BASELINE`. Current source/configuration, provisional geometry and affected interfaces require regression before the current revision receives equivalent `HARDWARE_PASS` or acceptance status.

## Current blockers

1. Clean unrestricted affected-package and complete Core/Edge target builds/tests are not recorded.
2. Physical geometry/footprint/sensor mounts are provisional and kinematic dimensions conflict with provisional URDF wheel centers.
3. Shared PCA9685 base/head chip-wide initialization/concurrency is unresolved.
4. Core/Edge safe-idle, network/time, hardware identity/freshness and restart are not current-source validated.
5. D435 live stream/VO/self-filter/voxel validation is not complete; voxel remains disabled.
6. Manual mapping, approved quality thresholds, immutable map/location release and rollback are not physically accepted.
7. Navigation tuning/routes/cancel/recovery/named-location acceptance is not complete.
8. Audio aliases/devices/acoustics, UI framebuffer/touch, power calibration and shutdown integration need target/hardware evidence.
9. Edge UI service ownership/dependency decision remains unresolved.

## Safety-critical measurement gaps

| Measurement gap | Impact |
| --- | --- |
| Chassis/wheels/sensor geometry, footprint and geometry digest | Blocks motion |
| Wheel signs, radius/kinematics, encoder CPR/scale and odometry drift | Blocks motion |
| Near-field thresholds and measured stop/slowdown distance/envelope | Blocks motion |
| Control/controller/recovery tuning and physical limits | Blocks motion/missions |
| Localization covariance/drift/dropout and navigation tolerances | Blocks mapping/navigation acceptance |
| RealSense mount and D435 self/floor filter bounds | Blocks voxel subsystem acceptance; does not block LiDAR-only non-motion validation |
| Battery voltage/percentage and low/critical thresholds | Blocks calibrated power/shutdown acceptance; does not block source validation |
| Head servo centers/soft limits/cable clearance and camera/tag mount | Blocks head/Scan360/semantic acceptance; shared PCA issue may block combined operation |
| LiDAR live rate/angle/mount and filter effect | Blocks mapping/navigation acceptance |
| Speech wake/VAD/latency/feedback and UI performance/touch | Blocks Edge subsystem acceptance; does not block Core non-motion validation |

No numeric threshold was invented for these gaps.

## Production-impact findings

| Severity | Finding | Affected files/packages | Impact / release block | Recommended follow-up |
| --- | --- | --- | --- | --- |
| High | Kinematic wheelbase/track `0.165/0.165 m` conflicts with provisional URDF wheel-center layout implying `0.230/0.200 m` | base/localization/description | Blocks motion, odometry, geometry digest and map/nav trust | Physically measure, choose reviewed source of truth, regenerate and regress |
| High | Base and head separately initialize shared PCA9685 `0x40` | `savo_base`, `savo_head` | Potential frequency/reset/concurrency interaction; blocks combined actuator acceptance | Define single shared-device contract/owner and test startup/restart/failure |
| Medium | Edge role unit depends on always-on UI runtime while distributed UI flag is false and setup does not install companion | systemd/UI/bringup | Startup warning/missing dependency or duplicate UI/framebuffer ownership | Decide owner, align renderer/unit/feature flag/setup, verify one-owner restart |
| Medium | D435 serial is hard-bound and RPLIDAR uses non-persistent `/dev/ttyUSB0` | realsense/lidar/deployment | Wrong/missing device can block or misbind sensor startup | Verify installed serial and add reviewed persistent identity policy |
| Medium | Repository does not supply the deployed ALSA alias/general hardware udev policy | speech/setup/deployment | Fresh Edge/audio/device access is not fully reproducible | Add target-verified alias/udev artifacts after hardware identity review |
| Low | Three legacy retired-package references remain in non-production metadata/fallback code | perception config, legacy UI Python, base Python docstring | Documentation noise/consumer inventory ambiguity; no current C++ authority impact | Remove through normal source review and rerun package/interface audits |

The managed-sandbox test limitations are an evidence-environment blocker, not a newly proven production implementation defect.

## Documentation-only findings corrected

- Added ten missing subsystem plans and all five mandatory governance documents.
- Rewrote six skeletal plans and retained/enhanced detailed control/description plans.
- Replaced stale component sheets and removed unsafe/invented launch/control guidance.
- Added `savo-ui-runtime.service` and its unresolved ownership finding to service docs.
- Added complete test/governance/audit indexes and Phase 7 status/changelog entries.
- Standardized result/status/evidence/abort/return-to-service language.

## Remaining documentation debt

No required Phase 7 document or unexplained link/command/package/interface/TF/hardware/storage/service mismatch remains. Future documentation work is evidence population and engineering decision closure: approved numeric acceptance criteria, geometry/calibration records, target execution results, hardware media/manifests, and issue references for the production findings.

## Final readiness statement

| Readiness dimension | Status |
| --- | --- |
| Documentation completeness | COMPLETE |
| Source/static validation | PASS |
| Hardware-free affected-package regression | BLOCKED by execution environment |
| Target Core/Edge/observer validation | NOT RUN for current revision |
| Hardware validation | Historical baseline only; current regression NOT RUN/BLOCKED |
| Integration validation | NOT RUN |
| Acceptance status | BLOCKED |
| Production motion authorization | NOT AUTHORIZED |

Phase 7 completed the verification system and repository-wide documentation audit. It did not complete the physical verification represented by that system.
