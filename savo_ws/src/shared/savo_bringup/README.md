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

## Guarded autonomous mapping bringup (AM-4)

The core-side autonomous mapping stack is composed with:

```bash
ros2 launch savo_bringup autonomous_mapping.launch.py \
  map_id:=campus_main
```

The launch starts base, LiDAR, range safety, control, localization, core power,
supervisor, live-map Nav2 and the package-local autonomous mapping launch. It
does not send an autonomous mission goal and defaults the control layer to
`STOP`.

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
  "{contract_version: 1, mission_id: mission_campus_main_001, actor_id: operator_1, map_id: campus_main, map_revision: 0, strategy: 1, auto_save: true, require_quality_approval: false, mission_timeout: {sec: 0, nanosec: 0}}"
```

The action goal is the only mission start boundary. Stable frontier exhaustion
triggers monitor-only mode, atomic map-session save, pose-graph serialization
and committed-session verification before success is reported.

`savo_description` is intentionally not included in AM-4. AM-0B will add the
final robot-state-publisher and description launch after the real dimensions,
sensor transforms and STL meshes are provided. Until then, mapping readiness
fails closed when the required TF chain is absent.

For a non-hardware launch inspection, each package group can be disabled with
its `start_*` argument. The production defaults start all core-side groups.
