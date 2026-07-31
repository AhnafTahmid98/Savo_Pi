# savo_mapping

`savo_mapping` is the Robot Savo ROS 2 Jazzy package for mapping-session
orchestration, mapping strategies, map saving and verification, saved-map
quality evaluation, production-map release, semantic-map support, and
exploration-goal selection.

The package is developed on Ubuntu 24.04 and validated on a PC before
deployment to Raspberry Pi 5 ARM64. Production implementation is C++17.
Python is limited to ROS launch and appropriate development tooling.

## Ownership and safety boundary

`savo_mapping` owns mapping workflow and map-data decisions. It does not own
robot hardware, base control, collision avoidance, localization, or Nav2
execution.

It must never publish:

```text
/cmd_vel
/cmd_vel_nav
/cmd_vel_safe
/goal_pose
/navigate_to_pose
```

The only permitted autonomous navigation handoff is:

```text
/savo_mapping/exploration/selected_goal
  -> exploration_goal_handoff_node
  -> /savo_nav/exploration/navigate_to_pose
```

Authority is divided as follows:

- `savo_nav` owns Nav2 and navigation execution.
- `slam_toolbox` owns map creation and `map -> odom`.
- `savo_localization` owns `odom -> base_footprint`.
- `robot_state_publisher` owns `base_footprint -> base_link`.
- `savo_mapping` monitors inputs and controls mapping workflow only.
- RealSense and voxel components here are for semantic mapping support and
  monitoring; they do not own the Nav2 local voxel obstacle layer.

The mapping supervisor must never bypass `savo_nav` or directly command motion.

## Implemented scope

The current implementation includes:

- Core mapping, exploration, workflow, session, and status contracts.
- Monitor-only mapping-supervisor operation.
- Manual online asynchronous `slam_toolbox` mapping.
- SLAM lifecycle health and manual-workflow state bridging.
- PC simulated scan, odometry, and TF input.
- Map-session save and pose-graph serialization.
- Saved-map artifact and manifest verification.
- Continued-mapping preflight and pose-graph resume.
- Saved-map quality evaluation and navigation approval/revocation metadata.
- Production-map release, verification, cataloging, and active-map promotion.
- Exploration-goal handoff to the dedicated `savo_nav` action, including
  rejection, abort, timeout, cancellation, retry, cancellation rejection,
  overdue cancellation, and terminal-result ownership handling.

## Current production-capable scope

The package now includes C++ implementations for:

- manual and continued SLAM Toolbox mapping
- mapping mode and workflow authority management
- mapping supervision and lifecycle health monitoring
- map-session save, verification, quality evaluation, release, and cataloging
- frontier detection, frontier selection, exploration management, and guarded
  exploration-goal handoff to `savo_nav`
- coverage planning, explicit execution approval, cancellation, and unified
  operation orchestration
- Scan360 planning and guarded rotation through `savo_control`
- mapped-location registration and semantic landmark core contracts
- semantic AprilTag prompt/event bridging and location-registry event mirroring

These paths are ready for staged Pi/robot validation. Hardware-specific tuning
values remain provisional until measured on the robot.

## Intentionally deferred or disabled paths

The following files remain non-production scaffolds and are not installed by
the package:

- unified package-level mapping bringup (`savo_bringup` will own full-system
  orchestration later)
- package-owned RealSense/voxel mapping monitors
- live map-quality scoring beyond the saved-map evaluator
- optional RViz presets that are not required on either Raspberry Pi

Zero-byte files must not be treated as implemented functionality.

## Semantic mapping and location registration

The semantic entrypoint is:

```bash
ros2 launch savo_mapping semantic_mapping.launch.xml \
  map_id:=campus_main \
  autostart:=true
```

It starts the normal manual SLAM workflow plus the two mapping-owned semantic
nodes:

- `semantic_landmark_bridge_node` observes `savo_head` confirmation hints and
  authoritative `savo_locations` events, then publishes read-only operator/UI
  events on `/savo_mapping/semantic_events`.
- `mapped_location_registration_node` owns the typed
  `/savo_mapping/locations/register` action. It obtains fresh AprilTag evidence
  from `savo_head`, asks `savo_supervisor` for authorization, validates the
  map-frame landmark and approach poses, and submits a pending candidate to
  `savo_locations`.

The bridge never persists locations and never commands motion. Legacy head
confirmation JSON is explicitly marked as hint-only; the typed registration
action always requests fresh confirmation evidence before persistence.

## Interface contract sources

Implemented and reserved contracts are maintained in:

- `config/topics.yaml`
- `include/savo_mapping/topic_names.hpp`
- `include/savo_mapping/exploration_goal_handoff.hpp`
- `include/savo_mapping/slam_lifecycle_topics.hpp`
- `include/savo_mapping/saved_map_quality.hpp`
- `include/savo_mapping/production_map_release.hpp`

### Exploration goal handoff

| Direction | Interface | Type |
| --- | --- | --- |
| Subscribe | `/savo_mapping/exploration/selected_goal` | `geometry_msgs/msg/PoseStamped` |
| Publish | `/savo_mapping/exploration_goal/state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/exploration_goal/status` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/exploration_goal/feedback` | `std_msgs/msg/String` |
| Service | `/savo_mapping/exploration_goal/cancel` | `std_srvs/srv/Trigger` |
| Action client | `/savo_nav/exploration/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

Only one exploration goal may be owned at a time. Goal ownership is released
only after the action reports a terminal result.

### Map-session saving

| Direction | Interface | Type |
| --- | --- | --- |
| Service | `/savo_mapping/map_session/save` | `std_srvs/srv/Trigger` |
| Service client | `/slam_toolbox/save_map` | `slam_toolbox/srv/SaveMap` |
| Service client | `/slam_toolbox/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` |
| Publish | `/savo_mapping/map_session/state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/map_session/result` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/map_session/manifest` | `std_msgs/msg/String` |

A saved session includes a map YAML file, map image, serialized pose graph,
pose-graph data, and a package-owned manifest.

### Saved-map quality and navigation approval

| Direction | Interface | Type |
| --- | --- | --- |
| Service | `/savo_mapping/map_quality/evaluate` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/navigation_handoff/approve` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/navigation_handoff/revoke` | `std_srvs/srv/Trigger` |
| Publish | `/savo_mapping/map_quality/state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/map_quality/evaluation` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/navigation_handoff` | `std_msgs/msg/String` |

Approval records downstream navigation readiness. It does not start Nav2 or
move the robot.

### Production map release and catalog

| Direction | Interface | Type |
| --- | --- | --- |
| Service | `/savo_mapping/map_release/create` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/map_release/verify` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/map_release/promote` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/map_release/deactivate` | `std_srvs/srv/Trigger` |
| Publish | `/savo_mapping/map_release/state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/map_release/result` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/map_catalog` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/active_map` | `std_msgs/msg/String` |

Promotion changes package-owned active-map metadata only. It does not configure
or launch Nav2.

### SLAM lifecycle health

| Direction | Interface | Type |
| --- | --- | --- |
| Service client | `/slam_toolbox/get_state` | `lifecycle_msgs/srv/GetState` |
| Publish | `/savo_mapping/slam_lifecycle_state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/slam_health` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/manual_workflow_state` | `std_msgs/msg/String` |

## Implemented executables

- `mapping_supervisor_node`
- `slam_lifecycle_health_bridge_node`
- `map_session_manager_node`
- `saved_map_verifier_node`
- `first_map_validator_node`
- `map_quality_evaluator_node`
- `map_catalog_manager_node`
- `exploration_goal_handoff_node`
- `simulated_slam_input_node` (PC development fixture)

The fake `savo_nav` exploration server is built only under `BUILD_TESTING`. It
is not installed and must never be used in production.

## Implemented launch files

- `monitor_only.launch.xml`
- `manual_mapping.launch.xml`
- `continued_mapping.launch.py`
- `map_session_manager.launch.xml`
- `map_quality_evaluator.launch.xml`
- `map_catalog_manager.launch.xml`
- `exploration_goal_handoff.launch.xml`
- `simulated_manual_mapping.launch.xml` (PC development)

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
set +u
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select savo_mapping \
  --symlink-install \
  --event-handlers console_direct+

source ~/Savo_Pi/savo_ws/install/setup.bash

colcon test \
  --packages-select savo_mapping \
  --event-handlers console_direct+

colcon test-result --verbose
```

A phase is complete only when the complete package suite reports zero errors,
zero failures, and zero skipped tests.

## Production constraints

- ROS 2 Jazzy on Ubuntu 24.04.
- C++17 production implementation.
- Lightweight and Raspberry Pi 5 ARM64 compatible.
- No test-only executable may be installed.
- No unfinished scaffold may be installed as a production artifact.
- Fake data and fake servers are permitted only for PC validation.
- Real-robot strategy profiles remain deferred until hardware validation.

## Phase 4L-B3G — unified Coverage operation orchestration

Production Coverage bringup uses `coverage_operation_orchestrator_node` as the
only public operator approval surface. The B3F approve, cancel and reset
services are remapped under `/savo_mapping/_internal/coverage_execution/*`.

The orchestrator requires both:

1. a staged valid Coverage plan in `coverage_execution_handoff_node`; and
2. a fresh, ready and authorized `/savo_supervisor/state_summary` snapshot.

Public operation interfaces:

| Direction | Name | Type |
| --- | --- | --- |
| Service | `/savo_mapping/coverage_operation/approve` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/coverage_operation/cancel` | `std_srvs/srv/Trigger` |
| Service | `/savo_mapping/coverage_operation/reset` | `std_srvs/srv/Trigger` |
| Publish | `/savo_mapping/coverage_operation/state` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/coverage_operation/status` | `std_msgs/msg/String` |
| Publish | `/savo_mapping/coverage_operation/events` | `std_msgs/msg/String` |

Publishing `/savo_mapping/coverage/path` remains planning output only. The
orchestrator does not subscribe to that path, own an action client, publish
velocity commands or bypass `savo_nav`. If supervisor authorization is lost
during an active Coverage mission, it can only request cancellation through
the internal B3F service.

### Supervisor-authorized location review gateway

`location_review_gateway_node` exposes
`/savo_mapping/locations/review` as the supported operator review boundary. It
loads the authoritative candidate from `/savo_locations/candidates/get`, checks
that the candidate is still pending at the expected revision, requests the
matching non-motion authorization from `savo_supervisor`, and then forwards
exactly one approval or rejection request to `savo_locations`.

The gateway publishes transient-local status on
`/savo_mapping/locations/review/status`, terminal JSON results on
`/savo_mapping/locations/review/results`, and a heartbeat on
`/savo_mapping/locations/review/heartbeat`. It owns no SQLite storage, map TF,
head motion, navigation goal, or velocity interface. Raw registry mutation
services remain package-internal integration boundaries; deployment-level ROS
security permissions are still required to prevent an unauthorized ROS process
from calling them directly.

### Operator candidate review CLI

`location_review_cli` is the supported keyboard/terminal fallback for location
candidate review. It lists the pending queue through
`/savo_locations/candidates/list`, inspects a candidate through the read-only
`/savo_locations/candidates/get` service, and sends approval or rejection only
to `/savo_mapping/locations/review`.

Examples:

```bash
ros2 run savo_mapping location_review_cli list
ros2 run savo_mapping location_review_cli inspect candidate-campus-main-27
ros2 run savo_mapping location_review_cli approve candidate-campus-main-27 \
  --actor operator-1
ros2 run savo_mapping location_review_cli reject candidate-campus-main-27 \
  --actor operator-1 --reason "duplicate doorway marker"
```

The CLI automatically loads the current candidate revision when `--revision`
is omitted. The gateway rechecks that revision before authorization, so a
candidate changed by another operator fails closed as stale. `--json` provides
schema-versioned machine-readable service responses; usage errors remain
non-zero process exits with a diagnostic message.

## Autonomous mapping mission foundation (AM-1)

`autonomous_mapping_orchestrator_node` exposes the typed
`/savo_mapping/autonomous/run` action, the typed
`/savo_mapping/autonomous/control` service and retained mission status. It
coordinates the existing mapping mode manager and guarded exploration handoff;
it does not send Nav2 goals directly. AM-3 calls only the public map-session
save service and never calls slam_toolbox save or serialization services
itself.

AM-1 supports one frontier mission at a time. Safety or readiness loss latches
a pause, cancels any active exploration handoff, and requires an explicit
resume.

## Stable frontier completion detection (AM-2)

`frontier_explorer_node` now publishes retained typed planner evidence on
`/savo_mapping/frontier_explorer/typed_status`. The autonomous orchestrator
uses a pure C++ completion detector that requires distinct planner sequences,
a configured stable duration, no pending/active handoff, fresh status,
readiness and an inactive safety stop.

Supported exhaustion evidence is `no_frontiers`,
`no_reachable_frontiers`, and optionally `no_selectable_frontier`; the last
is disabled by default because selector thresholds can temporarily reject
otherwise valid frontiers. Any later `goal_selected` plan revokes a pending
completion and returns the mission to exploration.

AM-2 completion evidence is still the only path into mission completion. With
`auto_save=false`, the mission completes successfully with an explicit
manual-save-required reason and `map_saved=false`.

## Automatic map save and verification (AM-3)

With `auto_save=true`, confirmed completion first switches the workflow to
monitor-only mode, then calls `/savo_mapping/map_session/save`. The existing
map-session manager remains the sole owner of occupancy-grid save, pose-graph
serialization, manifest creation, staging commit and overwrite policy.

After a successful service response, the orchestrator verifies the committed
session with the same `saved_map_contract` used by the standalone verifier.
Mission success is reported only after the grid YAML/image, posegraph, data and
manifest all pass verification. Save timeouts, service failures, malformed
responses and verification failures terminate with `RESULT_SAVE_FAILED`.
Quality evaluation, operator approval and production release remain separate
later phases.

## Package-local autonomous mapping composition (AM-4)

`autonomous_mapping.launch.xml` now composes only mapping-owned processes:
SLAM Toolbox, mapping readiness, map-session persistence, frontier selection,
the guarded exploration handoff, and the AM-1 through AM-3 mission
orchestrator. Hardware drivers, control, localization, supervisor policy and
Nav2 remain outside this package and are composed by `savo_bringup`.

The launch starts in `monitor_only` with an idle session. The frontier node's
static gate is enabled, but `exploration_manager_node` keeps runtime authority
closed until the typed autonomous mission action has passed readiness and
safety checks. Launching the stack therefore never starts robot motion.
