# savo_nav

## Purpose

Core Nav2 orchestration, verified-map activation, readiness, public goal admission/gateway, guarded exploration/coverage execution, recovery integration, and named-location navigation.

## Deployment

Core only. `savo_bringup` selects production saved-map, development saved-map, or live-mapping navigation. Production uses the active release contract.

## Responsibilities

- Start/configure Nav2 for live SLAM or saved AMCL map.
- Verify active map/release identity and synchronize supervisor context.
- Aggregate map/TF/odom/LiDAR/costmap/control/safety readiness.
- Admit one goal at a time; enforce source/map/readiness/timeouts/cancellation and fixed behavior-tree selection.
- Provide separate public navigation, mapping-exploration, and coverage action contracts.
- Resolve approved named locations, request supervisor authorization, navigate only to `approach_pose`, and optionally confirm arrival via head.

## Non-responsibilities and authority boundaries

Does not approve/map releases or locations, publish `/cmd_vel` or `/cmd_vel_safe`, execute motor output, own Core safety, or allow RViz goals in production. Nav2 output is `/cmd_vel_nav` and still passes control/perception/base.

## Package structure

C++ gateway/readiness/map-context/recovery/admission/named-location nodes and libraries, Nav2 YAML, behavior trees, launches, RViz/tools, tests.

## Runtime components

### Package nodes

`active_map_context_node` verifies `active_map.yaml` and release artifacts; `navigation_readiness_node` evaluates inputs; `control_recovery_guard_node` tracks control/recovery allowance; `goal_admission_gate_node` closes admission on policy loss; `goal_gateway_node` exposes public actions and forwards to internal Nav2; `navigate_to_location_node` owns semantic navigation.

### Nav2 components

`map_server` + AMCL for saved map, or SLAM Toolbox's live map; planner/controller/behavior/BT navigator/waypoint follower and lifecycle managers. DWB outputs remapped `/cmd_vel_nav`.

## Runtime data flow

```text
verified release/live map + localization + scan + safety/control
 -> readiness/admission -> public gateway -> internal Nav2
 -> /cmd_vel_nav -> savo_control -> perception gate -> base
named query -> locations resolve -> supervisor authorize -> same gateway -> optional head confirm
```

## ROS interfaces

### Published topics

`/savo_nav/{state,status,readiness,readiness_reason,current_goal,goal_source,result,recovery_state,heartbeat,markers}`; `/savo_nav/map_context/{status,heartbeat}`; navigation gateway state/status/feedback/result; control-recovery and goal-admission allowed/reason/status; `/cmd_vel_nav` (`Twist`). Nav2 publishes map/costmaps/plan/action feedback as configured.

### Subscribed topics

`/map`, `/map_metadata`, TF/static TF, `/odometry/filtered`, localization health, `/scan`, optional filtered D435 points and RealSense status, global/local costmaps, `/safety/stop`, slowdown, control mode/status/recovery, supervisor/map-context evidence.

### Services

Clients `/savo_locations/resolve` and `/savo_supervisor/authorize_location_operation`; Nav2 lifecycle/map services are internal. No generic public goal service.

### Actions

| Action | Type | Purpose |
| --- | --- | --- |
| `/savo_nav/navigation/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Public admitted mission/location gateway |
| `/savo_nav/exploration/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Mapping-only exploration gateway |
| `/savo_nav/coverage/execute_path` | `savo_msgs/action/ExecuteCoveragePath` | Mapping-only path execution |
| `/savo_nav/locations/navigate` | `savo_msgs/action/NavigateToLocation` | Semantic destination workflow |
| `/navigate_to_pose`, `/follow_path` | Nav2 actions | Internal backends only |

Internal phase-7B actions under `/savo_nav/_internal/...` are not public. Silent goal replacement is prohibited; cancellation acknowledgement is required.

## TF ownership

Saved-map AMCL owns `map -> odom`; live mapping SLAM Toolbox owns it. Localization owns `odom -> base_footprint`; description owns fixed TF. Never run AMCL and SLAM map authority concurrently.

## Parameters and configuration

LiDAR readiness defaults: pointcloud not required; odom/scan/safety timeout 1 s, control 2 s, costmaps 3 s, accepted mode `NAV`. Gateway disallows degraded readiness and behavior-tree override, allows coordinates within ±1000 m, execution 300 s and feedback stale 10 s, one active goal. Nav2 DWB caps x/y `0.24 m/s`, yaw `0.55 rad/s`, controller 15 Hz, footprint from description, inflation 0.45 m. Voxel overlay requires explicit D435 validation and fresh filtered cloud.

## Launch files

`production_navigation.launch.py` verifies active release then includes saved-map stack. `saved_map_navigation.launch.py` is controlled development input. `live_mapping_navigation.launch.py` omits map_server/AMCL and uses SLAM map authority.

## Persistent state and runtime files

Reads `/var/lib/robot_savo/maps/production/active_map.yaml` and referenced immutable map artifacts; does not write releases. Nav2 may retain normal logs/AMCL runtime state.

## Hardware ownership

None directly.

## Dependencies

### Internal Robot Savo dependencies

Description, LiDAR, localization, control, perception, mapping clients, locations, supervisor, head arrival confirmation, shared messages/bringup.

### External ROS/system dependencies

Nav2 planner/controller/BT/AMCL/map server/lifecycle, TF2, occupancy/nav/geometry messages.

## Safety behavior

Admission closes on unverified map/context, stale readiness, wrong control mode, safety stop, recovery conflict, busy gateway, invalid goal, or backend loss. External behavior-tree path selection is disabled. Cancellation is bounded and acknowledged; no path bypasses control/perception/base.

## Failure and degraded behavior

Goal rejection/abort/timeout/stale feedback/readiness loss cancels backend and publishes terminal reason. Optional D435 failure forces the validated LiDAR profile; it must not silently keep voxel readiness.

## Startup and shutdown behavior

Production verifies release/digests/context before lifecycle activation. Shutdown/cancel propagates to active backend before nodes stop.

## Build

`bash deploy/core/build_core.sh --clean --test`.

## Run

Production: `ros2 launch savo_nav production_navigation.launch.py active_map_contract:=...`; normally use `savo_bringup`.

## Validation and testing

Tests cover active-map parsing/digests, readiness, gateway source/admission/cancel/runtime, recovery guard, coverage proxy/follow-path, named resolution/authorization/arrival, launch/profile/BT contracts.

## Current validation status

Implemented/source-tested. Current verified production map, target Nav2 build, AMCL/localization, safety/control integration, tuning, cancellation/recovery, named navigation, and physical routes require integration/hardware validation.

## Known limitations and remaining validation

D435 voxel profile is blocked pending physical validation. Saved-map development launch is not production release verification. RViz is observer-only.

## Change-control considerations

Public/internal action separation, admission policy, TF owner, BT files, costmap sources, velocity/footprint, active-map verification, or cancellation semantics require full navigation/safety regression.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_nav/README.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Navigation test plan](../testing/navigation_test_plan.md)
- [Motion authority model](../architecture/motion_authority_model.md)
- [Ownership matrix](package_ownership_matrix.md)
