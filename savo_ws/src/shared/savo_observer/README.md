# Robot SAVO observer

`savo_observer` is the read-only visualization and telemetry package for an
external ROS 2 Jazzy desktop. It never runs robot hardware, sends goals,
changes modes, approves maps, resets safety, or commands velocity.

## Surfaces

- RViz2 provides maps, TF, geometry, paths, costmaps, scans, and optional point
  clouds. RViz2 is for Ubuntu desktop systems, including a bridged Ubuntu VM on
  a Mac. It does not run natively on a phone.
- The C++ telemetry node publishes a low-rate JSON snapshot plus ROS diagnostics.
- The C++ HTTP node serves a responsive dashboard and read-only telemetry and
  configuration endpoints. Phones and tablets use this browser dashboard.

The dashboard has no command routes or buttons. Its default bind address is
`127.0.0.1`. Binding to `0.0.0.0` is intended only for a trusted LAN protected
by the host firewall; never expose the port directly to the public Internet.
Subsystem cards show the original package-owned status text. When that text is
JSON or key/value telemetry, the browser extracts and graphs only real numeric
values it received; it reports numeric telemetry as unavailable otherwise.
Graph buffers and polling rates are bounded by the selected profile.

## Views

Available views are `overview`, `robot_model`, `tf`, `sensors`, `safety`,
`localization`, `mapping`, `manual_mapping`, `autonomous_mapping`, `coverage`,
`scan360`, `map_quality`, `navigation`, `costmaps`, `locations`, and
`full_debug`. The fixed frame is normally `map`; model/sensor-centric views use
`base_footprint`. High-bandwidth point clouds and camera images are disabled in
every checked-in view by default. The `full_debug` view brings together the
robot model, uncluttered TF, map, scan, three odometry sources, planar
localization markers, physical range markers, both point clouds, both camera
feeds, costmaps, plan, footprint, coverage path, and selected exploration goal
without adding command tools. Its fixed frame remains `map`, with the normal
camera target on `base_footprint`.

Six nonempty legacy RViz configurations were imported with their original
hashes recorded. The TF, costmap, and sensor views were then intentionally
corrected for current read-only coverage; the costmap correction removed
`SetGoal` and `SetInitialPose`. Four empty mapping placeholders were
implemented. The full inventory and hashes are in
`config/migration_manifest.yaml`.

## Planar localization visualization

RViz's built-in Odometry covariance display is disabled for filtered, wheel,
and visual odometry in `full_debug` and the focused localization/mapping views.
That display renders the full 3D covariance and produced the enormous
yellow/pink vertical volumes seen in VM testing. The odometry messages and
their covariance values are untouched; only that unsuitable rendering is
disabled.

`localization_visualizer_node` subscribes read-only to
`/odometry/filtered`, `/wheel/odom`, and `/vo/odom` using SensorData QoS. It
publishes at 5 Hz on `/savo_observer/localization_markers` and reuses five
marker IDs:

- The green filtered-pose arrow, blue wheel-pose arrow, and magenta VO-pose
  arrow show each source at its real X/Y position. Tiny Z offsets exist only
  to prevent z-fighting.
- The cyan filtered-pose ellipse is a 2-sigma confidence ellipse by default.
  The node symmetrizes the XY covariance block, computes its eigenvalues and
  eigenvectors, uses `2 * sqrt(eigenvalue)` as each principal radius, and
  rotates the ellipse by the major eigenvector. Invalid or
  non-positive-semidefinite covariance is not drawn. Extreme radii are clamped
  for RViz only.
- The amber arc uses `pose.covariance[35]`: its default half-span is
  `2 * sqrt(yaw_variance)`. A tighter arc means lower reported yaw uncertainty;
  a wider arc means higher uncertainty. Its visual span is bounded, and no Z,
  roll, or pitch covariance volume is created.

Use `view:=localization` to compare all three pose sources. VO remains opt-in
as an RViz Odometry display, while its lightweight marker appears whenever
`/vo/odom` exists. `view:=manual_mapping` keeps the map, model, LiDAR,
filtered pose, compact uncertainty, and physical range markers enabled; its
filtered D435 obstacle cloud remains an independent opt-in.

## Physical range and D435 obstacle layers

`range_visualizer_node` subscribes read-only to the three physical obstacle
sensors with their source SensorData/best-effort QoS:

- LEFT: `/savo_perception/range/left_m` in `tof_left_link`
- RIGHT: `/savo_perception/range/right_m` in `tof_right_link`
- FRONT: `/savo_perception/range/front_ultrasonic_m` in
  `ultrasonic_front_link`

It publishes exactly twelve reusable markers at 5 Hz on
`/savo_observer/range_markers`. Each fresh positive measurement is a thin ray
along the sensor frame's local +X axis, a small endpoint at the measured
distance, and a compact label. The existing TF tree supplies all mounting
position and direction geometry. Missing, stale, and invalid samples have
distinct neutral/orange status markers and no fabricated ray or obstacle
endpoint. These states describe observer data freshness only; the node does
not reproduce the perception safety algorithm or decide stop/slow behavior.

The three range markers are separate from the filtered D435 obstacle cloud on
`/savo_perception/obstacles/points`. The preferred sensor-debug combination is
the range marker layer plus that filtered cloud. The raw D435 cloud on
`/camera/camera/depth/color/points` is much heavier and remains a separate,
explicit opt-in.

## Profiles and bandwidth

- `low_bandwidth`: 0.5 Hz dashboard telemetry; edge and speech observations disabled.
- `standard`: 1 Hz telemetry and ordinary spatial displays.
- `full_debug`: 2 Hz telemetry; point clouds and costmaps remain manually enabled.
- `mobile`: 0.5 Hz telemetry, bounded 60-sample history, no images or clouds.

No sensor data or odometry is republished by `savo_observer`.
`enable_camera_preview`, `enable_pointclouds`, and
`enable_raw_d435_pointcloud` default to false.
Passing a flag as true creates one temporary runtime copy of the selected RViz
configuration and enables only the corresponding displays. Source-owned
`.rviz` files and QoS remain unchanged, the temporary copy is removed when
RViz exits, and the observer remains read-only.

The low-bandwidth marker nodes default on for RViz/full modes. They can be
disabled independently with `enable_localization_markers:=false` or
`enable_range_markers:=false`. Both use latest-sample semantics, bounded
marker counts, deterministic IDs, and low-rate timers; they do no image or
point-cloud processing and perform no TF polling.

The head-camera display consumes the existing image-transport endpoint at
`/savo_head/camera/image_raw/compressed` with Best Effort reliability; the
observer does not add a republisher. `enable_camera_preview:=true` enables both
`HeadCameraCompressed` and `D435ColorImage` when they are present in the
selected view. `d435_image_transport` defaults to `compressed` and accepts
only `raw` or `compressed`. D435 compressed mode uses the standard Jazzy RViz
`image_transport` contract at
`/savo_observer/d435/color/image_raw/compressed`; the `/compressed` suffix
selects compressed transport for the base topic
`/savo_observer/d435/color/image_raw`. Raw mode directly consumes the unchanged
production `/camera/camera/color/image_raw` topic. The optional relay that
creates the compressed copy runs on Edge under `savo_realsense`, not in this
package.

`enable_pointclouds:=true` enables observer-friendly clouds, with
`/savo_perception/obstacles/points` as the primary filtered D435 obstacle
cloud. It does not enable the large raw D435 cloud.
`enable_raw_d435_pointcloud:=true` is the separate diagnostic opt-in for
`/camera/camera/depth/color/points` in views that contain it. The filtered
cloud remains obstacle-only and marking-oriented; observer configuration does
not alter perception filtering or navigation clearing semantics. Compressed
image transport does not solve the separate sustained raw-pointcloud transport
limitation across Edge Wi-Fi to the bridged VM.

The Edge relay is disabled by default and exists because full-size raw D435
RGB DDS samples were unreliable across Edge Wi-Fi to the bridged Ubuntu VM.
It compresses only an observer copy and does not reduce the production
`640x480x30` D435 stream, alter the raw pointcloud, change TF ownership, or
affect navigation, perception, or VO. It does not address or hide the separate
RealSense startup firmware notification.

## Network setup

On the robot and observer use the same domain and compatible DDS implementation:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Both devices must be on the same multicast-capable LAN (or a routed DDS setup),
with synchronized clocks. A Mac Ubuntu VM must use bridged networking rather
than NAT. Allow DDS discovery/data and dashboard TCP port 8765 only on the
trusted LAN. `deploy/observer/check_connection.sh` reports required and optional
topic visibility and provides domain/firewall guidance when discovery is empty.

## Launch

```bash
ros2 launch savo_observer observer.launch.py mode:=full view:=overview profile:=standard
ros2 launch savo_observer observer.launch.py mode:=full view:=navigation profile:=standard
ros2 launch savo_observer observer.launch.py mode:=rviz view:=tf profile:=standard
ros2 launch savo_observer observer.launch.py mode:=rviz view:=sensors \
  profile:=standard enable_camera_preview:=true enable_pointclouds:=true \
  d435_image_transport:=compressed
ros2 launch savo_observer rviz_observer.launch.py view:=sensors \
  profile:=standard fixed_frame:=base_link enable_camera_preview:=true \
  enable_pointclouds:=true d435_image_transport:=compressed \
  use_sim_time:=false
ros2 launch savo_observer rviz_observer.launch.py view:=sensors \
  profile:=standard fixed_frame:=base_link enable_camera_preview:=true \
  enable_pointclouds:=true d435_image_transport:=raw use_sim_time:=false
ros2 launch savo_observer rviz_observer.launch.py view:=full_debug \
  profile:=standard fixed_frame:=map enable_camera_preview:=true \
  enable_pointclouds:=true enable_raw_d435_pointcloud:=true \
  d435_image_transport:=compressed use_sim_time:=false
ros2 launch savo_observer observer.launch.py mode:=dashboard profile:=mobile \
  dashboard_bind_address:=0.0.0.0 dashboard_port:=8765
```

For the mobile command, open `http://ROBOT_OBSERVER_PC_IP:8765` from a phone on
the same trusted LAN. The server runs on the external observer computer, not on
the robot Raspberry Pis.

Official RViz validation for Robot Savo will run in the bridged Ubuntu 24.04
Noble VM on the Mac. VM/runtime acceptance is not yet claimed here.

## Troubleshooting

- No topics: check `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, DDS implementation,
  multicast, VM bridge mode, and firewall.
- TF errors: use `view:=tf`; the observer never publishes transforms.
- Giant covariance volumes: use the checked-in views, which disable RViz's
  built-in 3D covariance and use the planar marker layer instead.
- Missing gray range marker: check the corresponding Float32 topic and sensor
  frame. Gray `STALE` means no fresh sample arrived within the observer
  timeout; orange `INVALID` reflects a non-finite or non-positive sample.
- Stale red panels: confirm the package owning that topic is enabled and online.
- Slow Wi-Fi: use `profile:=low_bandwidth`; leave clouds/images disabled.
- Dashboard unavailable: verify the bind address, TCP port, and local firewall.
