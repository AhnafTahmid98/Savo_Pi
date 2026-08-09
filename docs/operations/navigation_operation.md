# Navigation Operation

## Purpose and classification

> [!WARNING]
> An accepted destination can cause autonomous physical motion.

Production navigation uses the verified active release, public admitted
`savo_nav` gateway, control NAV lane, perception gate, and base watchdog.
Operators must not call internal Nav2 actions.

## Production preconditions

- Current geometry and footprint are approved for the site.
- `/var/lib/robot_savo/maps/production/active_map.yaml` and all referenced
  artifacts pass identity/hash verification.
- AMCL is the sole `map -> odom` authority; EKF owns
  `odom -> base_footprint`.
- Localization, scan, costmaps, control, safety, power, Nav2, supervisor, and
  map context are current and ready.
- For a named destination, its approved location is enabled and associated
  with the active map/revision/release.
- The operator has a working cancel/STOP control and clear observation.

Observe the production gateway before submitting a goal:

```bash
ros2 topic echo --once /savo_nav/readiness
ros2 topic echo --once /savo_nav/map_context/status
ros2 topic echo --once /savo_localization/health
ros2 topic echo --once /safety/stop
ros2 topic echo --once /savo_supervisor/system_ready
```

## Routine operation

1. Select an approved destination through the deployed typed bridge/operator
   client. UI presentation alone does not send a goal.
2. For named navigation, use the public
   `/savo_nav/locations/navigate` workflow. It resolves the approved
   `approach_pose`, requests supervisor authorization, and optionally confirms
   arrival with the head.
3. For an approved coordinate destination, use the public
   `/savo_nav/navigation/navigate_to_pose` client integration.
4. Confirm goal acceptance, active map identity, and control mode `NAV`.
5. Monitor progress, remaining distance, safety, localization, costmaps,
   recovery count, power, and people/obstacles.
6. On arrival, verify the terminal result and any required AprilTag arrival
   confirmation. Return to `STOP` when no next mission is authorized.

No routine command-line navigation client is installed by this repository.
Raw `ros2 action send_goal`, `/navigate_to_pose`, `/follow_path`, mapping-only
exploration, coverage, and `_internal` actions are developer/test interfaces.

## Cancellation and abort

Cancel through the same approved client and wait for acknowledgement. Then
confirm `STOP` if the mission is not being replaced:

```bash
ros2 run savo_control mode_cmd_cli.py STOP
ros2 topic echo --once /savo_control/mode_state
```

Abort for localization loss/jump, TF discontinuity, wrong map context, repeated
recovery or oscillation, persistent obstacle, power fault, unexpected speed,
Edge/client loss, service restart, or failed cancellation. If cancellation is
not acknowledged, use external/physical stop; do not submit another goal.

Goal rejection, no path, or invalid named location is a safe terminal result.
Correct the readiness, route, or map/location release; never bypass admission.
Development saved-map launch is not production navigation because it does not
provide the active-release verification pathway.
