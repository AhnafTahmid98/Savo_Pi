# savo_supervisor

## Purpose

Core robot-wide readiness, mission permission, startup arming, persistent system state, fault latch, map context, and controlled shutdown authority.

## Deployment

Shared source but Core build/runtime only; `supervisor.launch.py` and production systemd/bringup start `supervisor_node`.

## Responsibilities

Evaluate Core/Edge freshness/readiness, authorize typed operations, maintain map/release context, arm/disarm, latch faults, revoke authority, persist system generation/state, and publish shutdown requests.

## Non-responsibilities and authority boundaries

Does not plan/execute navigation, select frontiers, save maps, approve releases/locations, command motors, or bypass package readiness. It grants permission; operation owners execute/recheck it.

## Package structure

C++ policy/parsers/state store/node, YAML, systemd/scripts, and extensive contract/runtime tests.

## Runtime components

### `supervisor_node`

Only production executable. Aggregates component evidence, serves
authority/state services, persists state, and publishes capability/system
summaries. Mission executors continuously CHECK their bound lease and quiesce
when Supervisor revokes it.

## Runtime data flow

`component readiness + map context + persisted state -> authority/capabilities`; faults/staleness revoke permissions and may request cancellation/shutdown.

## ROS interfaces

### Published topics

`/savo_supervisor/{state_summary,capabilities,mission_authority,system_state,heartbeat}` and controlled shutdown/fault/edge summaries as configured; diagnostic output.

### Subscribed topics

Core/Edge bringup/readiness/heartbeats; control, safety, localization, mapping/nav mission, RealSense, speech, VO, UI, power, and component health topics with per-source timeouts.

### Services

| Service | Type | Purpose |
| --- | --- | --- |
| `/savo_supervisor/authorize_operation` | `savo_msgs/srv/AuthorizeOperation` | Typed mission operation permission |
| `/savo_supervisor/update_map_context` | `savo_msgs/srv/UpdateMapContext` | Store live/saved map identity only |
| `/savo_supervisor/authorize_location_operation` | `savo_msgs/srv/AuthorizeLocationOperation` | Location mutation/navigation permission |
| `/savo_supervisor/manage_system_state` | `savo_msgs/srv/ManageSystemState` | Status, arm, disarm, shutdown, clear latch |

### Actions

No action server. Supervisor revocation changes the authoritative lease state;
the bound autonomous-mapping executor detects that through its periodic CHECK,
cancels or pauses active downstream work, and requires explicit RESUME.

## TF ownership

None.

## Parameters and configuration

`config/supervisor.yaml` defines required components/timeouts, startup policy, authorization, persisted path, and mission capability. Edge speech/VO/UI are not required for startup by default. Deployment passes `/var/lib/robot_savo/supervisor/system_state.json`.

## Launch files

`supervisor.launch.py` loads policy and state path.

## Persistent state and runtime files

Atomically maintained system state/generation and fault/shutdown/arming context under the configured supervisor state path.

## Hardware ownership

None; shutdown execution remains privileged deployment responsibility.

## Dependencies

### Internal Robot Savo dependencies

Nearly all Core/Edge readiness producers and `savo_msgs`; mapping/nav/head/control as authorized executors.

### External ROS/system dependencies

ROS clients/actions, JSON/filesystem persistence.

## Safety behavior

Starts unarmed; stale/malformed/mismatched evidence, fault latch, wrong map context, or unsafe state denies/revokes. Fault clear requires explicit safe request. Shutdown service publishes request rather than executing shell commands.

## Failure and degraded behavior

Supervisor loss removes authorization; executors must cancel/stop on stale authority. Optional Edge feature loss degrades capability without granting motion.

## Startup and shutdown behavior

Loads/validates persistent state, evaluates readiness, requires explicit arm. Controlled shutdown revokes missions, latches state, persists, and publishes a deployment-layer request.

## Build

`bash deploy/core/build_core.sh --clean --test`.

## Run

`ros2 launch savo_supervisor supervisor.launch.py`.

## Validation and testing

Policy/parsers, freshness, readiness, mission/location authority, map context, transition/state store, edge supervision, runtime/service contracts.

## Current validation status

Implemented/source-tested; current two-Pi startup, arm/disarm, latch/recovery, revoke/cancel, persistence, and shutdown require target/integration validation.

## Known limitations and remaining validation

Readiness depends on string/JSON component contracts and physical evidence not available in source-only tests.

## Change-control considerations

Required-source lists, timeouts, authorization policy, persistent schema, cancellation set, or arming behavior are safety-critical.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_supervisor/README.md)
- [Safety architecture](../architecture/safety_architecture.md)
- [Motion authority model](../architecture/motion_authority_model.md)
- [Supervisor test plan](../testing/supervisor_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
