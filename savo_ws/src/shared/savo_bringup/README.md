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
