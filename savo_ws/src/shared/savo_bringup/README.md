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

The launch starts base, LiDAR, range safety, control, localization, core power,
supervisor, `savo_head`, live-map Nav2 and the package-local autonomous mapping
launch. It does not send an autonomous mission goal and defaults the control
layer to `STOP`. Scan360 is available through its public service but is started
only by the typed mission sequencer.

During a controlled real-robot test, first confirm mapping, navigation, safety,
localization and power readiness. Then request `NAV` control authority and send
the typed mission action with the same map identifier:

```bash
ros2 topic pub --once \
  /savo_control/mode_cmd \
  std_msgs/msg/String \
  "{data: NAV}"

ros2 action send_goal \
  /savo_mapping/autonomous/run \
  savo_msgs/action/RunAutonomousMapping \
  "{contract_version: 2, mission_id: mission_campus_main_001, actor_id: operator_1, map_id: campus_main, map_revision: 1, strategy: 1, auto_save: true, require_quality_approval: true, mission_timeout: {sec: 0, nanosec: 0}}"
```

The action goal is the only mission start boundary. AM-5 records the initial
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
its `start_*` argument. The production defaults start all core-side groups.

## AM-8 one-launch completion contract

`autonomous_mapping.launch.py` starts the location review gateway and the
mapping orchestrator in the same guarded stack. A production mission goal must
use `contract_version: 2` and `require_quality_approval: true`.

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
profile. The D435 obstacle pipeline is hardware validated, so canonical
real-robot Edge bringup defaults `d435_voxel_validated:=true` and
`start_obstacle_cloud:=true`. It starts the single validated D435 camera,
monitor and health nodes, front-depth producer, VO, local raw pointcloud,
filtered obstacle cloud, and compressed observer color relay. Explicitly
disabling the obstacle or observer relay remains available for controlled
testing. Helper loss does not gate the required LiDAR navigation path, and
LiDAR remains the reliable clearing source.

The first controlled real-robot test should use:

```bash
ros2 launch savo_bringup saved_map_navigation.launch.py \
  bringup_profile:=lidar_only \
  control_startup_mode:=STOP
```

The C++ `bringup_readiness_node` publishes:

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

It aggregates the supervisor, navigation, bridge, RealSense, VO, and speech
readiness required by the selected host and profile. It does not replace the
safety, supervisor, AM-8, or Nav2 authorities.

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
