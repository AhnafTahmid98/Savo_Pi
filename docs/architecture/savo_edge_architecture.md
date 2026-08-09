# Savo Edge Architecture

Edge is the perception and interaction computer. Its production set contains `savo_msgs`, `savo_description`, `savo_bringup`, `savo_bridge`, `savo_perception`, `savo_power`, `savo_realsense`, `savo_speech`, `savo_ui`, and `savo_vo`.

## Data paths

```text
RealSense D435 -> savo_realsense -> RGB-D topics -> savo_vo -> /vo/odom -> Core EKF (optional)
                              `-> optional obstacle cloud -> Core/Nav2 profile (gated)

ReSpeaker <-> savo_speech <-> /run/savomind/speech.sock <-> SavoMind
ROS state <-> savo_bridge <-> /run/savo_bridge/* <-> SavoMind
ROS state -> savo_ui (read-only)
```

The repository-bound D435 configuration selects serial `801212070967`, depth `848 x 480 @ 30 Hz`, color `640 x 480 @ 30 Hz`, aligned depth, and synchronized point-cloud profile. These values describe configuration, not proof that the installed device/USB path meets them. RealSense TF publication is disabled; shared description owns its fixed frames.

## Authority boundary

Edge can observe robot state and request only bridge operations with explicit typed adapters, bounds, peer-credential checks, timeouts, and current Core readiness/authority. It cannot publish arbitrary ROS commands, write motors, mutate supervisor state directly, approve maps/locations, or bypass navigation admission. UI is read-only. Returned SavoMind speech audio is bounded and validated before playback.

## Startup and failures

Default Edge deployment starts the bridge, RealSense, and VO; speech, UI, and D435 obstacle cloud are disabled unless selected. Edge publishes a separate `/savo_bringup/edge/*` readiness namespace. Required missing/stale services block the selected profile; optional failures produce degraded state. Losing Edge must result in cancellation/timeout or reduced capability, never a Core motion bypass.

Bridge snapshots and sockets are volatile under `/run/savo_bridge`. Speech uses `/run/savomind/speech.sock`. ROS logs default to the Edge user's ROS log tree unless the deployed service overrides it. No authoritative map, location, supervisor, or approval state is stored on Edge.

Source-contract tests cover bridge and speech boundaries, UI mutation rejection, and Edge launch membership. Live D435 USB/RGB-D/VO quality, ReSpeaker/playback, display/touch, socket permissions, time sync, and two-host failure behavior remain hardware/integration gates.
