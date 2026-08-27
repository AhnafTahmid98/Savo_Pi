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
robot model, TF, map, scan, three odometry sources, both point clouds, both
camera feeds, costmaps, plan, footprint, coverage path, and selected
exploration goal without adding command tools.

Six nonempty legacy RViz configurations were imported with their original
hashes recorded. The TF, costmap, and sensor views were then intentionally
corrected for current read-only coverage; the costmap correction removed
`SetGoal` and `SetInitialPose`. Four empty mapping placeholders were
implemented. The full inventory and hashes are in
`config/migration_manifest.yaml`.

## Profiles and bandwidth

- `low_bandwidth`: 0.5 Hz dashboard telemetry; edge and speech observations disabled.
- `standard`: 1 Hz telemetry and ordinary spatial displays.
- `full_debug`: 2 Hz telemetry; point clouds and costmaps remain manually enabled.
- `mobile`: 0.5 Hz telemetry, bounded 60-sample history, no images or clouds.

No sensor data is republished by `savo_observer`. `enable_camera_preview`,
`enable_pointclouds`, and `enable_raw_d435_pointcloud` default to false.
Passing a flag as true creates one temporary runtime copy of the selected RViz
configuration and enables only the corresponding displays. Source-owned
`.rviz` files and QoS remain unchanged, the temporary copy is removed when
RViz exits, and the observer remains read-only.

The head-camera display always consumes `/savo_head/camera/image_raw` using raw
transport. `d435_image_transport` defaults to `compressed` and accepts only
`raw` or `compressed`. Compressed mode uses the standard Jazzy RViz
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
  profile:=standard fixed_frame:=base_link enable_camera_preview:=true \
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
- Stale red panels: confirm the package owning that topic is enabled and online.
- Slow Wi-Fi: use `profile:=low_bandwidth`; leave clouds/images disabled.
- Dashboard unavailable: verify the bind address, TCP port, and local firewall.
