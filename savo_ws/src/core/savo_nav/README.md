# savo_nav

`savo_nav` is Robot Savo's core-side ROS 2 Jazzy navigation package. It owns
Nav2 orchestration, guarded goal admission, saved-map and live-mapping
navigation, navigation readiness, bounded recovery coordination, named-location
navigation, and coverage-path forwarding.

Production runtime code is C++17. Python is used only for ROS launch, tests, and
the isolated location-integration smoke tool. The deployment target is the
Raspberry Pi 5 ARM64 `savo-core` computer.

## Safety and ownership boundary

Nav2 may publish velocity only through:

```text
Nav2 -> /cmd_vel_nav -> savo_control -> /cmd_vel
     -> savo_perception safety gate -> /cmd_vel_safe -> savo_base
```

`savo_nav` must never publish directly to `/cmd_vel`, `/cmd_vel_safe`, or
`/cmd_vel_recovery`.

The public action servers are protected by the control/recovery guard and the
goal-admission gate. The internal gateway action names are remapped under
`/savo_nav/_internal/...`; callers must use the public guarded interfaces.

## TF authority

Saved-map navigation:

```text
map --AMCL--> odom --savo_localization--> base_footprint
    --robot_state_publisher--> base_link
```

Live-mapping navigation:

```text
map --SLAM Toolbox--> odom --savo_localization--> base_footprint
    --robot_state_publisher--> base_link
```

AMCL and SLAM Toolbox must never publish `map -> odom` at the same time.

## Implemented runtime

The package currently provides:

- `navigation_readiness_node`
- `control_recovery_guard_node`
- `goal_admission_gate_node`
- `goal_gateway_node`
- `navigate_to_location_node`
- guarded `NavigateToPose` forwarding for mission/location goals
- guarded exploration-goal forwarding for `savo_mapping`
- guarded coverage-path validation and Nav2 `FollowPath` forwarding
- goal arbitration, cancellation acknowledgement, watchdogs, and late-result
  handling
- saved-map Nav2 bringup with AMCL and map server
- live-mapping Nav2 bringup without AMCL or map server

## Launch files

### Saved-map navigation

```bash
ros2 launch savo_nav saved_map_navigation.launch.py \
  map:=/absolute/path/to/map.yaml \
  map_id:=campus_main \
  autostart:=true
```

This launch starts map server, AMCL, planner, controller, behavior server, BT
navigator, waypoint follower, lifecycle managers, readiness, the guarded goal
gateway, control/recovery guard, and goal-admission gate.

### Live-mapping navigation

```bash
ros2 launch savo_nav live_mapping_navigation.launch.py \
  autostart:=true
```

Use this only while SLAM Toolbox is already publishing `/map` and `map -> odom`.
It intentionally does not start AMCL or map server. This is the Nav2 execution
path required by frontier exploration during an active mapping session.

Before enabling motion, run the read-only graph preflight:

```bash
ros2 run savo_nav run_mapping_nav_preflight --mode live
```

After deliberately changing `savo_control` to `NAV`, require the complete
readiness chain before sending a goal:

```bash
ros2 run savo_nav run_mapping_nav_preflight --mode live --expect-ready
```

## Readiness profiles

`config/readiness.yaml` is the guarded LiDAR-only baseline used for the first
real-robot navigation tests. It requires map, TF, fused odometry, LiDAR, Nav2,
global/local costmaps, NAV control mode, and fresh safety state. It does not
require the filtered D435 cloud because the active baseline costmaps do not yet
consume that cloud.

`config/readiness_realsense_voxel.yaml` requires
`/savo_perception/obstacles/points`. Use it only after the real D435 frame,
freshness, self-filter, floor, and obstacle tests pass and the Nav2 voxel layer
is enabled.

The raw RealSense topic must never be consumed directly by Nav2. The filtered
cloud has obstacle-only semantics, so its future voxel layer must use marking
without clearing; LiDAR remains responsible for reliable clearing.

## Build and test

Development/test build:

```bash
cd ~/Savo_Pi/savo_ws
set +u
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-up-to savo_nav savo_mapping \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash

colcon test \
  --packages-select savo_nav savo_mapping \
  --event-handlers console_direct+

colcon test-result --verbose
```

Production Pi build after the full test suite has passed elsewhere:

```bash
colcon build \
  --packages-up-to savo_nav savo_mapping \
  --cmake-args -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
```

Disabling `BUILD_TESTING` prevents test fixtures from entering the Pi install.

## First guarded Pi test order

1. Start robot description, base, localization, LiDAR, perception safety, and
   `savo_control`.
2. Confirm the TF chain and `/odometry/filtered` are healthy.
3. Start manual SLAM Toolbox mapping from `savo_mapping`.
4. Start `live_mapping_navigation.launch.py`.
5. Change `savo_control` to `NAV` only after the area is physically secured.
6. Confirm `/savo_nav/readiness` becomes `ready` before sending any goal.
7. Test a short guarded exploration goal, cancellation, safety stop, and stale
   dependency behavior before enabling autonomous frontier iteration.

## Remaining hardware gates

Code readiness does not replace physical validation. Before final deployment,
measure the actual robot footprint and validate DWB behavior, goal tolerances,
acceleration limits, recovery distances, AMCL tuning, and the optional D435
voxel-layer self-filter on the real floor.
