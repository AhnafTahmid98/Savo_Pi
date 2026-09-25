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

The production Core autonomous mapping stack is started from the repository
root through the ownership-protected runner:

```bash
bash deploy/core/run_autonomous_mapping.sh map_id:=campus_main
```

Direct `ros2 launch savo_bringup autonomous_mapping.launch.py` remains a
component/development interface and does not provide the production Core-owner
lock.

### Temporary Core LiDAR-only degraded mapping

`core_lidar_mapping_degraded` is an explicit, temporary real-robot test mode
for the current unreliable ToF wiring. It has no required short-range sensors:
both ToFs, front depth, and front ultrasonic remain optional. The
ToF driver and diagnostics stay enabled, so invalid readings remain invalid
and visible; this profile does not invent safe distances. LiDAR, SLAM,
localization, Nav2, base/control, power, mapping health, and mapping authority
remain required and fail closed through their existing policies.

The production runner cannot select the degraded profile: it pins
`autonomous_mapping_profile:=production` and rejects profile or paired-file
overrides. From a controlled Core shell, the degraded profile must instead be
selected explicitly:

```bash
ros2 launch savo_bringup autonomous_mapping.launch.py \
  map_id:=<lowercase_map_id> \
  autonomous_mapping_profile:=core_lidar_mapping_degraded \
  perception_use_ultrasonic:=false
```

The launch still starts in `STOP`, does not ARM, and does not submit an
autonomous-mapping action, Nav2 Spin action, navigation goal, or velocity
command. Dedicated autonomous mapping fixes `initial_scan360_required:=false`;
the launch fails before nodes start if an operator tries to enable it, and the
production runner rejects that override. Start-pose capture and SLAM startup
are stationary. Mission admission acquires mapping-local authority and selects
NAV without commanding base motion. Frontier planning begins from the current
TF pose, and the first navigation goal comes from the selected reachable
frontier through the exploration handoff. Nav2 may still turn normally while
following that real path; the degraded profile retains `max_vel_theta: 0.30`,
`acc_lim_theta: 0.50`, and `decel_lim_theta: -0.50` rather than forcing
straight-line motion. Its DWB controller also enables the holonomic
`Twirling` critic at scale `10.0` so travel-time pure rotation is penalized;
zero translational speed remains legal, angular correction while moving stays
available, and `RotateToGoal` remains enabled for required final alignment.
Before any motion-capable action, an operator must
inspect the live sources and routing:

```bash
ros2 topic hz /scan
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic echo --once /savo_perception/range_health
ros2 topic echo --once /savo_perception/tof_status
ros2 topic echo --once /safety/stop
ros2 topic info --verbose /cmd_vel_nav
ros2 topic info --verbose /cmd_vel_safe
```

An invalid optional ToF is expected to remain visible as a sensor-level
`ERROR`; it must not be the sole reason for required-range failure or
`/safety/stop`. Any real LiDAR, localization, SLAM, Nav2, control, power,
mapping-health, authority, or ownership failure remains a no-go condition.

The launch staggers description, base, LiDAR, range safety, control,
localization, and core power to reduce Core Pi startup contention. The
autonomous branch then uses observed readiness, not elapsed time: stable Core
and localization inputs release the SLAM foundation; SLAM ACTIVE plus fresh
scan, filtered odometry, map, and map-to-odom release Nav2; active Nav2
lifecycle nodes and action servers release the inert mapping runtime. It does
not send an autonomous mission goal and defaults the control layer to `STOP`.
Head, semantic/location, VO, ultrasonic, coverage and initial/final scan
workflows remain available but default off for the first Core-only geometric
mapping run.
The dedicated launch defaults `start_supervisor=false`; normal system bringup
still supports the system Supervisor with its existing defaults and latch policy.

During a controlled real-robot test, first confirm mapping, navigation, safety,
localization and power readiness, then send one typed mission action.
No Supervisor ARM or service is required. Generation zero asks the orchestrator
to acquire a mission-bound mapping-local lease from direct subsystem evidence
before it selects any motion-capable control mode:

```bash
ros2 action send_goal \
  /savo_mapping/autonomous/run \
  savo_msgs/action/RunAutonomousMapping \
  "{contract_version: 3, mission_id: mission_campus_main_001, actor_id: operator_1, map_id: campus_main, map_revision: 1, strategy: 1, authority_request_id: mapping_request_001, authority_generation: 0, require_semantic: false, auto_save: false, require_quality_approval: false, mission_timeout: {sec: 0, nanosec: 0}}"
```

The action goal is the only mission start boundary, but it proceeds only after
the orchestrator acquires and verifies its exact mapping-local lease. Nonzero
pre-acquired Supervisor generations are rejected explicitly; callers must send
zero, keeping request, actor, map and semantic fields intact. AM-5 records the
initial map-frame pose without moving the base. The dedicated workflow never
performs startup Scan360 and enters frontier exploration directly. A typed
control request can insert
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
use `contract_version: 3`, `authority_generation: 0` for local acquisition, and
`require_quality_approval: true`.

See [mapping-local authority validation](../../../../docs/validation/mapping_local_authority.md)
for ownership, compatibility, and Core ROS 2 Jazzy validation commands.

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

The production Core entry point is:

```bash
bash deploy/core/run_core.sh
```

Direct `robot_bringup.launch.py` and `core_bringup.launch.py` invocation is for
intentional component/development testing and does not replace the production
ownership runner.

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

Canonical non-autonomous Core startup uses ordinary package launch includes
with bounded, historical compatibility offsets. Package-owned health and
Supervisor authority remain responsible for runtime eligibility. Dedicated
autonomous mapping is the exception: it uses the package-owned staged
readiness coordinator for process release only.
The robot still starts with control in `STOP`, Supervisor unarmed, and no
submitted action goal.

The historical delay arguments remain declared and forwarded for launch API
compatibility:

- `description_start_delay_s` (default `0.0`)
- `base_start_delay_s` (default `5.0`)
- `lidar_start_delay_s` (default `10.0`)
- `perception_start_delay_s` (default `15.0`)
- `control_start_delay_s` (default `20.0`)
- `localization_start_delay_s` (default `30.0`)
- `power_start_delay_s` (default `35.0`)
- `head_start_delay_s` (default `40.0`)
- `supervisor_start_delay_s` (default `45.0`)
- `location_lifecycle_start_delay_s` (default `50.0`)
- `manual_mapping_start_delay_s` (default `60.0`)
- `navigation_start_delay_s` (default `55.0`)
- `readiness_start_delay_s` (default `60.0`)

`savo_bringup/startup_timing.py` owns the shared defaults used by
`robot_bringup.launch.py`, `core_bringup.launch.py`, and the direct autonomous
mapping launch. All offsets are seconds from launch, not cumulative waits.
The autonomous launch uses `mapping_start_delay_s` only as the earliest time at
which dependency evaluation begins. It does not use
`navigation_start_delay_s` to release Nav2. The Core wrapper preserves its
historical `readiness_start_delay_s` alias for that evaluation offset; the
robot role wrapper exposes it as `core_readiness_start_delay_s`. Edge timing
remains independent.

These offsets spread startup load but never prove health or authorize motion.
Existing `start_*` flags omit disabled optional components.

Safe idle remains stopped and unarmed, starts no SLAM or Nav2, does not center
the head, and does not auto-start its scan. Manual SLAM and saved-map
navigation start only in their matching dependency stages. Location services
remain off in normal safe idle and do not imply navigation.

The dedicated `autonomous_mapping.launch.py` composes the current SLAM, Nav2,
and mapping runtime as separate readiness-gated groups. With
`start_navigation:=false start_mapping:=true`, only the stationary SLAM
foundation is released; mapping runtime remains omitted because its Nav2
dependency is intentionally absent. With both flags false, neither mapping nor
Nav2 is launched. `start_navigation:=true start_mapping:=false` is rejected as
an invalid live-mapping composition. Launch never submits the typed
autonomous-mapping action; mission readiness and mapping-local lease admission
remain separate and strict.

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

The C++ `bringup_readiness_node` can publish:

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

It is not launched by normal production Core or Edge bringup. Dedicated
autonomous mapping launches it as a process-sequencing observer; it does not
replace the safety, Supervisor, mapping, or Nav2 authorities and cannot select
a control mode, submit an action, or publish velocity.

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
