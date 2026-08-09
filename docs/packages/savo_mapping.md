# savo_mapping

## Purpose

Core owner of live mapping workflow, frontier/coverage planning and guarded handoff, autonomous mapping mission, map-session save/verification/quality, semantic candidate orchestration, and operator-approved production release.

## Deployment

Core only. Manual and autonomous compositions are selected by `savo_bringup`; production state uses `/var/lib/robot_savo/maps` paths passed by deploy scripts.

## Responsibilities

- Coordinate SLAM Toolbox without taking its TF authority.
- Manage mapping mode/readiness/workflow/session state.
- Run manual mapping and frontier exploration.
- Plan coverage and stage it until explicit supervisor-authorized execution.
- Execute contract-v2 `RunAutonomousMapping`: start pose, initial/final Scan360/head scans, frontier exhaustion, coverage, return near start, save, verification, location checks, operator review, and release.
- Register semantic candidates via head evidence and locations persistence.
- Save artifacts, verify them, score quality, catalog/release production maps, and maintain active-map contract.

## Non-responsibilities and authority boundaries

Does not drive motors, publish `/cmd_vel*`, execute Nav2 paths, own `map -> odom`, approve maps/locations for the operator, or bypass supervisor/control/perception. Selected goals/plans are inert until admitted by `savo_nav` and the command chain.

## Package structure

Production C++ domain/orchestrator/planner/nodes, XML/Python launches, YAML, RViz, CLI review tool, and broad unit/runtime/contract tests.

## Runtime components

### Workflow, SLAM, save, and release

`mapping_mode_manager_node`, `mapping_supervisor_node`, `slam_lifecycle_health_bridge_node`, `map_session_manager_node`, `saved_map_verifier_node`, `map_quality_evaluator_node`, `map_catalog_manager_node`, and `first_map_validator_node`.

### Exploration and autonomous mission

`frontier_explorer_node` detects/selects map frontiers; `exploration_manager_node` grants runtime planning authority; `exploration_goal_handoff_node` sends selected goals only through the guarded nav exploration action; `autonomous_mapping_orchestrator_node` owns mission sequencing, timeouts, cancellation, save, review, and AM-8 release.

### Coverage and Scan360

`coverage_mapper_node` produces inert `nav_msgs/Path`; `coverage_execution_handoff_node` stages a plan; `coverage_operation_orchestrator_node` rechecks supervisor state and explicitly approves/cancels internal handoff; `scan360_mapper_node` uses `RotateToHeading` and map/scan quality. Publishing a path never starts motion.

### Semantic locations

`semantic_landmark_bridge_node`, `semantic_interruption_coordinator_node`, `mapped_location_registration_node`, `location_review_gateway_node`, and operator `location_review_cli`. Mapping validates/georeferences; `savo_locations` persists; supervisor authorizes; operator approves/rejects.

### Non-production components

`simulated_slam_input_node` and fixtures/fake servers are test-only.

## Runtime data flow

```text
/scan + odometry/TF -> SLAM Toolbox -> /map (SLAM owns map->odom)
/map -> frontier/coverage planner -> selected goal/path (inert)
    -> mapping handoff -> savo_nav admission -> savo_control -> safety -> base
session -> map_saver -> verify/quality -> operator review -> joint map/location release
```

## ROS interfaces

### Published topics

Key stable outputs include `/savo_mapping/{mode,workflow_phase,session_state,readiness}`, frontier state/status/typed status and `/exploration/selected_goal`, exploration runtime authority/handoff state, coverage path/state/status/execution/operation state/events, Scan360 state/status, semantic interruption/status, `/autonomous/status` (`AutonomousMappingStatus`), joint active release, and map-session/quality/catalog status. `/map` is published by SLAM Toolbox, not package nodes.

### Subscribed topics

`/map`, `/scan`, `/odometry/filtered`, TF, safety stop, localization/control/supervisor readiness/authority, head scan/tag state, nav handoff feedback/results, locations events/status, and SLAM lifecycle state.

### Services

Key services: mapping mode/session start/cancel; `/savo_mapping/map_session/save`; frontier/exploration cancel; coverage request/reset, staged approve/cancel/reset, supervisor-facing coverage-operation approve/cancel/reset; Scan360 start/cancel; semantic submission/review gateway; `/savo_mapping/autonomous/control`; `/autonomous/review_release`; plus SLAM lifecycle/map-saver clients. Names are configuration-driven under `/savo_mapping`.

### Actions

| Action | Type | Direction/purpose |
| --- | --- | --- |
| `/savo_mapping/autonomous/run` | `RunAutonomousMapping` | Server: full guarded mission |
| `/savo_mapping/locations/register` | `RegisterMappedLocation` | Server: typed candidate workflow |
| `/savo_nav/exploration/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Client: selected frontier handoff |
| `/savo_nav/coverage/execute_path` | `ExecuteCoveragePath` | Client: guarded coverage |
| `/savo_control/rotate_to_heading` | `RotateToHeading` | Client: Scan360 |
| `/savo_head/apriltag/confirm` | `ConfirmAprilTag` | Client: semantic evidence |

## TF ownership

SLAM Toolbox launched by this package owns dynamic `map -> odom` during live mapping. Mapping nodes only look up frames (`map`, `odom`, `base_link`, `laser_frame`); localization owns `odom -> base_footprint`, description owns fixed TF. Saved-map Nav2/AMCL must not run concurrently as map TF owner.

## Parameters and configuration

Important defaults: frontier `enabled=false` unless autonomous composition supplies runtime authority; 1 s planning period; map/TF stale bounds; coverage `auto_plan=false`, 0.5 m track spacing, unknown disallowed; handoff server/goal/cancel bounds 3/3/5 s; autonomous mission requires start pose, initial/final Scan360/head scans, coverage and return, locked geometry, operator approval timeout 600 s; save 45 s; coverage execution 900 s; completion requires 3 exhaustion observations stable for 5 s. Production roots are overridden to `/var/lib/robot_savo/maps/{sessions,production,release_transactions}`.

## Launch files

Production compositions: `manual_mapping.launch.xml`, `autonomous_mapping.launch.xml` and orchestrator, `frontier_mapping`, coverage mapper/handoff/operation, Scan360, semantic mapping, session/quality/catalog, continued mapping, monitor-only. `simulated_manual_mapping` and PC RViz are development only.

## Persistent state and runtime files

Session directories contain map YAML/image and manifests/verification/quality evidence. Production releases and `active_map.yaml` live under `/var/lib/robot_savo/maps/production`; joint release journals support rollback. Location snapshots remain owned by `savo_locations`.

## Hardware ownership

None directly; consumes LiDAR/localization/head and requests movement through guarded owners.

## Dependencies

### Internal Robot Savo dependencies

Messages, LiDAR, localization, head, control action, perception safety, nav actions, locations services, supervisor authority, description geometry, bringup.

### External ROS/system dependencies

SLAM Toolbox, Nav2 map saver/lifecycle, TF2, occupancy/nav/sensor messages, filesystem/digest handling.

## Safety behavior

Readiness/authority/safety loss cancels active handoffs; every motion stage has response/execution/feedback/cancel watchdogs. Map/location release requires verified artifacts, correct context/digest, locked geometry, explicit operator review, transactional rollback. Mapping never directly commands motors.

## Failure and degraded behavior

Stale map/TF/feedback, rejected goal, save/quality/location/release failure, timeout, or cancellation produces a terminal typed result and attempts bounded cancellation/rollback. No failed/unreviewed session becomes active production map.

## Startup and shutdown behavior

Starts monitor/manual state unless autonomous action accepted with authority. Cancellation propagates to nav/head/control dependencies. Shutdown cancels active operations and leaves sessions unreleased unless transaction completed.

## Build

`bash deploy/core/build_core.sh --clean --test`.

## Run

Use `savo_bringup` manual/autonomous launch, not isolated motion nodes for production.

## Validation and testing

Extensive C++/Python tests cover modes/workflow, frontiers/exhaustion, handoff/watchdogs, coverage planning/execution/authority, Scan360, start-pose TF, semantics/review, session save/artifacts, quality, autonomous sequencing/runtime/cancel, and AM-8 joint release/rollback.

## Current validation status

Implemented and source/integration-contract tested. Strategy tuning, current physical manual mission, guarded autonomous mission, save/verify/quality, semantics, operator review, and production release require hardware/integration validation.

## Known limitations and remaining validation

Production action admits FRONTIER with AM-7 coverage pass; standalone COVERAGE is not admitted. Planned waypoint/area strategies are not implemented public mission strategies. Default map-catalog YAML contains developer `~/Savo_Pi/runtime/maps` values but production bringup overrides them.

## Change-control considerations

Action versions, completion criteria, authority/watchdogs, TF roles, storage schema/roots, quality thresholds, and release transaction order require end-to-end rollback and physical regression.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_mapping/README.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Mapping test plan](../testing/mapping_test_plan.md)
- [Recovery operations](../deployment/recovery_operations.md)
- [Ownership matrix](package_ownership_matrix.md)
