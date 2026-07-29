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
- `savo_localization` owns `odom -> base_link`.
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

The validated baseline before Phase 4J-A3 is 352 tests with zero errors, zero
failures, and zero skipped tests.

## Deferred functionality

Zero-byte files for unfinished features are scaffolds, not production
implementations. Deferred areas include:

- Frontier detection, selection, and exploration.
- Exploration and mapping-mode managers.
- Autonomous and unified mapping bringup.
- Coverage and Scan360 mapping.
- Semantic landmark workflow integration.
- RealSense and voxel mapping monitors.
- Live map-quality scoring.
- RViz mapping configurations.
- Real-robot strategy profiles.

Deferred scaffolds must not be built or installed until their implementation
phase is complete and validated.

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
|---|---|---|
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
