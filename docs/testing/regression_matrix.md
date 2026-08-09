# Robot Savo Regression Matrix

## Purpose

This matrix defines the minimum defensible retest when source, configuration, hardware, deployment, interfaces or persistent contracts change. The change reviewer may increase scope. A small diff that changes behavior is not R0.

## Severity

| Level | Meaning | Minimum evidence |
| --- | --- | --- |
| R0 | Documentation-only correction with no behavioral/config/interface effect | Link/command/consistency checks and reviewer confirmation |
| R1 | Isolated source/unit regression | Affected static/unit/source-contract tests and build where applicable |
| R2 | Target component regression | R1 plus intended-host build/runtime, fault/restart and interface evidence |
| R3 | Subsystem integration regression | R2 plus every directly coupled producer/consumer and authority/freshness path |
| R4 | Physical robot regression | R3 plus non-actuating then staged physical tests under the applicable risk controls |
| R5 | Full acceptance/release requalification | Full staged system plan and acceptance decision for the affected release scope |

Use the highest severity reached by any consequence. Changes to safety gates, actuator mapping, TF/geometry, production release integrity or mission authority are never R0.

## Change-triggered minimum scope

| Change area | Severity | Minimum tests | Integration tests | Physical retest | Release impact |
| --- | --- | --- | --- | --- | --- |
| Documentation prose/link only | R0 | Link, command and contradiction audit | None unless meaning changed | None | No release identity change |
| `savo_base` source | R3–R4 | BAS-001–004 and affected package tests | CTL-005–007, PER-003, SUP authority path | BAS-005–009 applicable | Motion release blocked until passed |
| Motor PCA/GPIO/PWM/channel/address | R4–R5 | BAS-001–005; ownership audit | Head PCA concurrency, power, STOP/watchdog | All wheel directions, clamp, shutdown, guarded motion | Requalify motion; R5 if board/topology changes |
| Wheel inversion/sign | R4 | BAS-001–007, LOC-001/005 | Control/perception command trace | Every direction wheels-raised; short floor | Blocks motion |
| Wheel radius/diameter | R4–R5 | DSC-004–009, BAS-001, LOC-001/002 | Footprint/map/locations/nav context | Odom scale/drift and guarded nav | New geometry digest; assess remap/re-release |
| Wheelbase/track/encoder scale/CPR | R4–R5 | DSC/LOC static and unit tests | EKF, map/odom TF, mapping/nav | Straight/strafe/yaw scale/drift | Geometry and map compatibility review |
| `savo_control` mode/mux/shaper | R3–R4 | CTL-001–008 | SUP revoke, PER gate, BAS watchdog | CTL-009–010 applicable | Blocks motion profiles |
| Recovery/stuck/approach/rotate | R3–R4 | CTL-008 plus action/unit tests | Mapping/nav/head dependencies | Guarded recovery/rotation | Requalify affected mission |
| Core safety threshold/stale policy | R4–R5 | PER-001–005 | CTL/base/supervisor/readiness | Measured obstacle/stop envelope | Safety acceptance invalidated |
| ToF replacement/move/mux channel | R4 | PER-001–006, DSC geometry | Safety side identity/UI/supervisor | Fixtures and wheels-raised/floor gate | Blocks motion until thresholds reviewed |
| Ultrasonic replacement/move/GPIO | R4 | PER-001–006, DSC geometry | Safety/front-direction gate | Fixtures and stop behavior | Blocks motion |
| LiDAR replacement/device/config | R3–R5 | LID-001–007 | Mapping/Nav2/TF/readiness | Live scan, fixtures, mapping/nav | Requalify maps/nav if perception changed materially |
| IMU replacement/orientation/mode | R4–R5 | LOC-001–008, DSC TF | EKF/control/mapping/nav | Orientation, drift, guarded courses | Blocks localization and missions |
| Encoder hardware/GPIO | R4 | LOC-001–006 | Base signs/EKF/readiness | Each channel wheels-raised and measured course | Blocks motion/localization |
| Localization EKF config/covariance/masks | R3–R4 | LOC-001–004/007/008 | Control/mapping/nav/TF | Drift/dropout/course when behavior changes | Requalify navigation/mapping scope |
| Geometry profile or digest algorithm | R4–R5 | DSC-001–010 | Base/localization/TF/footprint/cloud/map/location/nav | Physical measurements and affected sensors/motion | New digest; old releases presumed incompatible until proven |
| URDF/Xacro/static TF | R3–R5 | DSC-002/003/007–010 | All source frames, localization, SLAM/AMCL | Mount/optical/footprint verification | Map/location release compatibility review |
| Nav2 footprint/padding | R4–R5 | DSC-009, NAV static/unit | Costmaps/planning/BT | Guarded clearance/routes | Requalify nav; remap if physical envelope changed |
| Head servo/PCA/limits/center | R4 | HED-001–007 | Base shared-PCA and TF | Conservative center/limits/scan | Blocks head/Scan360/tag acceptance |
| Pi Camera/mount/backend/profile | R3–R4 | HED-001–007, DSC mount | AprilTag/mapping/locations | Image/optical/tag evidence | Requalify semantic workflows |
| RealSense unit/cable/USB/serial | R3–R4 | RLS-001–007 | VO/PER cloud/readiness | Live profile/loss/restart | Blocks VO/cloud; nav voxel disabled |
| RealSense mount/profile/alignment | R4–R5 | DSC TF, RLS-001–007 | VO and D435 self-filter | Depth/alignment/drift/cloud | New geometry digest when mount changes; review map/nav |
| VO source/config/covariance/frames | R3–R4 | VO-001–008 | LOC optional fusion/readiness | Drift/motion/dropout | Fusion remains disabled until VO-009 |
| Enabling/changing VO fusion | R4–R5 | VO-009, LOC-001–008 | Mapping/nav/control/readiness | Full drift/dropout/guarded routes | Requalify localization-driven missions |
| D435 self/floor/range filter | R4 | PER-007–010 | RLS, Nav2 voxel costmap | Real mount/cloud/obstacle cases | Voxel profile blocked until passed |
| `savo_mapping` source/config | R3–R5 | MAP-001–007 | Supervisor/control/nav/locations | MAP-008–014 affected | Requalify mapping/release scope |
| Map quality thresholds/evaluator | R3–R5 | MAP-002/005/006/010 | Review/release/nav | Representative real maps | Existing releases require policy compatibility review |
| Map release/digest/active-context implementation | R5 | MAP-005–007/014, NAV-003/007 | Locations/supervisor/deploy backup | Verified map/navigation regression | Migrate/re-release or explicitly preserve old schema |
| `savo_locations` or DB schema | R3–R5 | LCT-001–009 | Mapping review/nav resolve/bridge/supervisor | LCT-010/named navigation as affected | Migration, backup/restore and location re-release |
| Location normalization/approval/release | R3–R5 | LCT-004–010 | Mapping/operator/nav/bridge | Named route when semantics change | Existing release compatibility/approval review |
| `savo_nav` source/gateway/admission | R3–R5 | NAV-001–008 | Control/perception/supervisor/locations | NAV-009–014 affected | Requalify navigation acceptance |
| Nav2 parameters/velocity/tolerances | R4–R5 | NAV static/unit, CTL/PER limits | Costmaps/controller/recovery | Guarded goals/cancel/recovery/routes | Blocks production nav until tuned evidence |
| Behavior tree files/selection | R3–R5 | NAV-001/002/004/006 | Gateway/recovery/control | Guarded goal/failure cases | Requalify affected navigation behavior |
| `savo_supervisor` policy/source/state schema | R3–R5 | SUP-001–008 | Every authorized executor/bridge/power | SUP-009/010 if missions affected | Authority acceptance invalidated |
| `savo_bridge` schema/operation/security | R3–R5 | BRD-001–008 | SavoMind/Core operation owner | BRD-009 only for affected live operation | Boundary/security review; full acceptance for new motion authority |
| `savo_speech` audio/runtime/protocol | R2–R4 | SPH-001–009 | SavoMind/UI/bridge | Audio hardware cases | Requalify speech; no motion release unless command acknowledgement affected |
| SavoMind protocol/model/provider | R3–R5 | BRD/SPH protocol/fault tests | Supervisor/operation admission/UI | Affected live typed operation only after gates | Compatibility/security review; R5 for authority semantics |
| `savo_ui` source/state/freshness | R2–R3 | UI-001–007 | Real telemetry/stale/safety states | Display/touch only | No motion release unless authority violation; mutation is R5 architecture issue |
| `savo_observer` source/config/RViz | R2–R3 | OBS-001–007 | Production network/telemetry | Workstation only | Any mutation interface is release blocker |
| `savo_power` calibration/threshold/source | R3–R4 | PWR-001–008 | Supervisor/bringup/UI/shutdown consumer | Live electrical/calibration/fault | Requalify low-power/shutdown scope |
| Network addressing/firewall/rpfilter | R3–R5 | BRG-006/009/010, OBS-005/007 | DDS, bridge/SavoMind, readiness | Two-Pi non-motion then missions | Blocks distributed acceptance |
| Chrony/time configuration | R3–R4 | Timestamp/source audits | RLS/VO/localization/TF/DDS | Sensor correlation/drift where affected | Blocks time-sensitive integration |
| Systemd unit/restart/ownership | R2–R5 | Render/verify, BRG-005/007–010 | Runtime dirs, duplicate owners, safe shutdown | Non-motion targets; actuator owner changes R4/R5 | Deployment release blocked until service evidence |
| Deployment/build/update scripts/role arrays | R2–R5 | Validators, role clean build/test | Complete role graph/storage/systemd | Safe-idle on affected roles | Requalify deployment; R5 if owners/defaults change |
| ROS distribution/domain/RMW/QoS | R3–R5 | Complete builds/tests/interface/QoS | Core/Edge/observer discovery/freshness | Affected sensor/motion integrations | Full distributed acceptance normally required |
| Persistent path/storage schema/permissions | R3–R5 | Storage/service/backup validators | Mapping/locations/supervisor/logs | No motion unless mission state affected | Migration/backup/rollback and release review |
| Backup/restore/rollback tools | R3 | Isolated backup/restore integrity tests | Mapping/location/supervisor/deployment | None by default | Recovery readiness invalid until passed |
| `savo_msgs` message/service/action | R3–R5 | Interface build/generation and all producers/consumers | Bridge, launch/runtime, SavoMind relation where applicable | Affected physical operation | Coordinated compatibility release; no partial deployment |

## Geometry, map and location rule

The controlled chain is:

```text
measured geometry -> canonical geometry digest -> sensor TF + Nav2 footprint
                  -> D435 self-filter -> map release -> location release
                  -> production navigation context
```

A material chassis, wheel, footprint or sensor-mount change creates a new digest. Regenerate/validate URDF, fixed TF, footprint and D435 filter; assess kinematics and every affected sensor. An old map/location release is not assumed valid. The release reviewer must either demonstrate compatibility with evidence and re-associate/re-release it, or create a new map and location release. Navigation stays blocked until active release verification succeeds.

## Interface-change rule

For every `savo_msgs` change:

1. inventory the exact interface and wire/semantic compatibility;
2. find every producer, consumer, launch/config and package manifest reference;
3. rebuild/test `savo_msgs` and all dependents on each role;
4. test bridge/SavoMind compatibility where the semantic boundary is related;
5. prevent mixed incompatible Core/Edge deployments;
6. rerun the owning subsystem and physical operation if behavior/authority changed.

## When R5 is mandatory

Use full acceptance for a production hardware revision, safety/STOP path change, actuator board/topology change, mission-authority boundary change, material geometry/footprint change with uncertain map compatibility, release-integrity/schema change, incompatible interface rollout, new motion-capable bridge operation, ROS/platform migration, or combined changes whose interactions cannot be bounded defensibly.

