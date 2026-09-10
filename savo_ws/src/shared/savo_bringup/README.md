# savo_bringup

`savo_bringup` owns cross-package production launch wiring. It does not own
location persistence, AprilTag interpretation, mapping validation, supervisor
policy, or robot motion.

## Typed location lifecycle

```text
savo_head ConfirmAprilTag
        -> savo_mapping RegisterMappedLocation
        -> savo_locations pending candidate
        -> savo_mapping authorized review gateway
        -> savo_supervisor authorization
        -> savo_locations approved LocationRecord
        -> savo_locations ResolveLocation
        -> savo_nav NavigateToLocation
        -> validated navigation action / approach_pose
        -> savo_head ConfirmAprilTag(CONFIRM_ARRIVAL)
```

Launch the production lifecycle layer:

```bash
ros2 launch savo_bringup location_integration.launch.py
```

The launch starts the persistent registry, supervisor authorization, head
observation/action nodes, mapped-location registration, the authorized review
gateway, and semantic navigation. Nav2 remains owned by the normal navigation
bringup and must expose `/savo_nav/navigation/navigate_to_pose`.

Run the hardware-independent production-launch runtime test after building with
test fixtures enabled:

```bash
ros2 run savo_bringup run_location_lifecycle_runtime
```

The runtime uses synthetic AprilTag observations and a fake Nav2 action server,
but all location, supervisor, mapping, head-confirmation, launch, persistence,
resolution, and semantic-navigation nodes are the real production nodes. It
writes permanent logs, a SQLite database, and a JSON report under
`~/Savo_Pi/runtime/location_lifecycle_phase2d/`.

## Guarded autonomous mapping bringup and sequencer (AM-5)

The core-side autonomous mapping stack is composed with:

```bash
ros2 launch savo_bringup autonomous_mapping.launch.py \
  map_id:=campus_main
```

The launch staggers description, base, LiDAR, range safety, control,
localization, core power, Supervisor, live-map Nav2, SLAM and autonomous
mapping to reduce Core Pi startup contention. It does not send an autonomous
mission goal and defaults the control layer to `STOP`. Head, semantic/location,
VO, ultrasonic, coverage and initial/final scan workflows remain available but
default off for the first Core-only geometric mapping run.

During a controlled real-robot test, first confirm mapping, navigation, safety,
localization and power readiness, arm Supervisor explicitly, and send one typed
mission action. Generation zero asks the orchestrator to acquire the exact
mapping lease before it selects any motion-capable control mode:

```bash
ros2 action send_goal \
  /savo_mapping/autonomous/run \
  savo_msgs/action/RunAutonomousMapping \
  "{contract_version: 3, mission_id: mission_campus_main_001, actor_id: operator_1, map_id: campus_main, map_revision: 1, strategy: 1, authority_request_id: mapping_request_001, authority_generation: 0, require_semantic: false, auto_save: false, require_quality_approval: false, mission_timeout: {sec: 0, nanosec: 0}}"
```

The action goal is the only mission start boundary, but it proceeds only after
the orchestrator acquires and verifies the exact Supervisor lease. Existing
callers that supply a nonzero, pre-acquired generation retain the independent
lease CHECK path. AM-5 records the initial
map-frame pose, runs an initial Scan360, switches to monitor-only for the initial
head scan, then enters frontier exploration. A typed control request can insert
a guarded conditional Scan360 and automatically resume frontier exploration.
Stable frontier exhaustion still triggers monitor-only mode, atomic map-session
save, pose-graph serialization and committed-session verification before
success is reported.

A conditional Scan360 can be requested only while the mission is actively
exploring:

```bash
ros2 service call \
  /savo_mapping/autonomous/control \
  savo_msgs/srv/ControlAutonomousMapping \
  "{contract_version: 2, mission_id: mission_campus_main_001, actor_id: operator_1, command: 4, reason: map_growth_stalled}"
```

The sequencer first cancels or waits out the guarded navigation handoff, enters
monitor-only, runs Scan360 through `/savo_mapping/scan360/start`, and returns to
frontier mode. AprilTag interruption, coverage, return-to-start, final scans,
operator approval and joint map/location release are implemented through AM-8.

`savo_description` is included before motion-capable components. Production
launch requires a locked geometry profile and fails closed on the checked-in
provisional profile. Controlled bench tests may explicitly set
`allow_provisional_geometry:=true`; this does not constitute a measurement lock.

For a non-hardware launch inspection, each package group can be disabled with
its `start_*` argument. The dedicated launch starts the required Core geometric
mapping groups and leaves optional hardware and semantic workflows disabled.

## AM-8 one-launch completion contract

`autonomous_mapping.launch.py` starts the location review gateway and the
mapping orchestrator in the same guarded stack. A production mission goal must
use `contract_version: 3`, an exact live Supervisor authority generation, and
`require_quality_approval: true`.

The required terminal path is:

```text
Mapping
→ save
→ map verification
→ quality evaluation
→ location verification
→ correlated operator approval
→ atomic joint map/location release
→ real map_release_id
```

The first real navigation test uses `savo_nav/config/nav2_saved_map.yaml`.
`nav2_saved_map_voxel.yaml` remains disabled until the filtered D435 point cloud
is validated on Robot SAVO.

## Full distributed Robot Savo bringup

Production runtime nodes are C++. Python is used only for ROS 2 launch
orchestration and the retained location-lifecycle test tool.

The primary entry point is:

```bash
ros2 launch savo_bringup robot_bringup.launch.py \
  host_role:=core \
  robot_mode:=safe_idle \
  bringup_profile:=lidar_only
```

The default Core `safe_idle` contract starts robot description/fixed TF, base,
LiDAR, perception, control in `STOP`, localization with VO fusion, Core power,
Supervisor without auto-arm, and the head with ROS camera transport. It does
not start mapping, Nav2, a navigation goal, or location lifecycle services.
The location lifecycle is owned by `manual_mapping`, `autonomous_mapping`, and
`saved_map_navigation`, or may be requested explicitly in another mode.

The Core operational baseline includes local geometry, base, control/safety,
LiDAR, perception, localization, power, and Supervisor contracts. Edge-owned
bridge, RealSense, VO, obstacle-cloud, speech, and UI resources are not Core
startup dependencies. The head starts in normal safe idle for camera and status
availability, with `center_on_start=false`, but is not a basic driving
dependency; semantic mapping modes own the location/head integration they
require.

### Simple Core startup

Canonical Core startup uses ordinary package launch includes with bounded,
historical compatibility offsets. Package-owned health and Supervisor
authority remain responsible for runtime eligibility; no global readiness
coordinator releases processes and there is no artificial `complete` stage.
The robot still starts with control in `STOP`, Supervisor unarmed, and no
submitted action goal.

The historical delay arguments remain declared and forwarded for launch API
compatibility:

- `description_start_delay_s` (default `0.0`)
- `base_start_delay_s` (default `3.0`)
- `lidar_start_delay_s` (default `6.0`)
- `perception_start_delay_s` (default `9.0`)
- `control_start_delay_s` (default `12.0`)
- `localization_start_delay_s` (default `17.0`)
- `power_start_delay_s` (default `22.0`)
- `head_start_delay_s` (default `27.0`)
- `supervisor_start_delay_s` (default `33.0`)
- `location_lifecycle_start_delay_s` (default `37.0`)
- `manual_mapping_start_delay_s` (default `40.0`)
- `navigation_start_delay_s` (default `40.0`)
- `readiness_start_delay_s` (default `45.0`)

These offsets spread startup load but never prove health or authorize motion.
Existing `start_*` flags omit disabled optional components.

Safe idle remains stopped and unarmed, starts no SLAM or Nav2, does not center
the head, and does not auto-start its scan. Manual SLAM and saved-map
navigation start only in their matching dependency stages. Location services
remain off in normal safe idle and do not imply navigation.

The dedicated `autonomous_mapping.launch.py` directly composes the current
Nav2, SLAM, and mapping runtime. Launch initializes them but never submits the
typed autonomous-mapping action; mission readiness and Supervisor lease
admission remain separate and strict.

Run the matching edge stack on `savo-edge`:

```bash
ros2 launch savo_bringup robot_bringup.launch.py \
  host_role:=edge \
  robot_mode:=safe_idle \
  bringup_profile:=lidar_only \
  start_speech:=false \
  start_ui:=false
```

Supported robot modes are:

| Mode | SLAM | AMCL/Nav2 | Motion startup |
|---|---:|---:|---|
| `safe_idle` | No | No | `STOP` |
| `manual` | No | No | Explicit operator control |
| `manual_mapping` | Yes | No | Explicit operator control |
| `autonomous_mapping` | Yes | Live-map Nav2 | Typed AM action only |
| `saved_map_navigation` | No | Verified production Nav2 | Explicit `NAV` authority |
| `diagnostics` | No | No | Motion components suppressed |

Supported profiles are `bench`, `lidar_only`, `lidar_d435_voxel`, and
`production`. Motion-capable non-bench profiles require a locked geometry
profile. Canonical `lidar_only` Edge bringup defaults
`d435_voxel_validated:=false` and `start_obstacle_cloud:=false`; it may still
start the D435 camera, front-depth producer, and VO. The filtered obstacle
cloud is explicitly enabled only for controlled validation or the
`lidar_d435_voxel` profile. Camera health
uses the lightweight front-depth, VO-health, and obstacle-cloud-health signals;
it does not add Image or PointCloud2 subscriptions to the production graph.
Explicitly disabling the obstacle or observer relay remains available for
controlled testing. When the validated obstacle cloud is started, its package
health and heartbeat remain observable; it remains independent of the Core
LiDAR navigation path, where LiDAR is the reliable clearing source.

### Simple Edge startup

Canonical Edge startup uses ordinary package includes with bounded
compatibility offsets to spread D435, VO, PointCloud2, Bridge, and optional-app
startup load. There is no global stage gate or synthetic completion state;
each producer publishes its own health and status.

Raw PointCloud2 remains part of the single validated RealSense driver:
`/camera/camera/depth/color/points` is transformed and filtered into
`/savo_perception/obstacles/points`. This Robot Savo perception voxel reduction
is not the Nav2 voxel costmap layer and Edge bringup does not start Nav2.

The historical delay arguments remain available for compatibility:

- `realsense_start_delay_s` (default `0.0`)
- `camera_support_start_delay_s` (default `10.0`)
- `vo_start_delay_s` (default `22.0`)
- `obstacle_cloud_start_delay_s` (default `34.0`)
- `observer_relay_start_delay_s` (default `40.0`)
- `speech_start_delay_s` (default `46.0`)
- `ui_start_delay_s` (default `50.0`)
- `bridge_start_delay_s` (default `54.0`)
- `readiness_start_delay_s` (default `60.0`)

These offsets spread the heavy D435, VO, filtered PointCloud2/voxel, and Bridge
startup work across roughly one minute. `readiness_start_delay_s` remains a
compatibility-only launch argument; production Edge does not launch the retired
bringup readiness node. Elapsed time therefore does not prove readiness or
authorize motion. The nested RealSense launch retains its own camera/support
offsets and component health remains authoritative.

The normal hardware-validation command is:

```bash
ros2 launch savo_bringup robot_bringup.launch.py \
  host_role:=edge \
  robot_mode:=safe_idle \
  bringup_profile:=lidar_d435_voxel \
  d435_voxel_validated:=true \
  start_realsense:=true \
  start_vo:=true \
  start_obstacle_cloud:=true \
  enable_observer_color_relay:=true \
  start_bridge:=true \
  start_edge_power:=false \
  start_speech:=false \
  start_ui:=false
```

`start_edge_power:=false` omits the known-bad Edge UPS node. Bridge graph
evidence also excludes both the UPS and the retired Edge readiness node. The
bridge proves its local Edge DDS presence from its own node and four owned
topics, while independently requiring Core graph visibility, fresh observation
topics, live command transport, and snapshot publication. There is therefore
no `Bridge -> Edge readiness -> Bridge` cycle in production.

During normal Ctrl+C shutdown, child nodes are terminated by the launch
service. Speech and UI retain their existing optional behavior.

For hardware discovery checks, avoid leaving the ROS 2 CLI daemon competing
with the sensor pipeline. Stop it first and request direct discovery:

```bash
ros2 daemon stop
ros2 node list --no-daemon
ros2 topic list --no-daemon
```

Run high-bandwidth `ros2 topic hz` probes one at a time because each probe adds
a live subscriber. Robot launch and readiness use native ROS subscriptions and
never invoke the ROS 2 CLI or depend on its discovery cache.

The first controlled real-robot test should use:

```bash
ros2 launch savo_bringup saved_map_navigation.launch.py \
  bringup_profile:=lidar_only \
  control_startup_mode:=STOP
```

The retained diagnostic-only C++ `bringup_readiness_node` can publish:

```text
/savo_bringup/core/state
/savo_bringup/core/ready
/savo_bringup/core/heartbeat
/savo_bringup/core/diagnostics

/savo_bringup/edge/state
/savo_bringup/edge/ready
/savo_bringup/edge/heartbeat
/savo_bringup/edge/diagnostics
```

It is not launched by production Core, Edge, or autonomous-mapping bringup.
It remains available for historical diagnostics and tests and does not replace
the safety, Supervisor, mapping, or Nav2 authorities.

Saved-map production navigation preserves the complete gate:

```text
AM-8 active release
→ release artifact integrity verification
→ geometry-profile verification
→ supervisor map-context synchronization
→ navigation readiness
→ guarded goal admission
→ Nav2
```
