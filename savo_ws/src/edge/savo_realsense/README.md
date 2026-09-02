# Robot Savo RealSense

## Production observer color relay

The production D435 color stream remains full-rate `640x480x30` at
`/camera/camera/color/image_raw` on Edge. For the external observer only,
The canonical `realsense_bringup.launch.py` starts one standard
`image_transport` republisher on Edge by default:

```text
/camera/camera/color/image_raw
  -> JPEG compression on Edge
  -> /savo_observer/d435/color/image_raw/compressed
  -> Ubuntu RViz compressed transport
```

Production depth at `/camera/camera/depth/image_rect_raw` and the raw cloud at
`/camera/camera/depth/color/points` also remain unchanged and available.

The relay exists only for the observer path. It does not change camera FPS or
resolution, the raw color topic, raw pointcloud, navigation or perception
semantics, VO, or fixed D435 TF ownership. It can be disabled for controlled
testing with `enable_observer_color_relay:=false`.

Normal production is:

```bash
ros2 launch savo_realsense realsense_bringup.launch.py
```

Production starts `camera_health_node` as the aggregate pipeline health
authority. It does not subscribe to raw Image, CameraInfo, or PointCloud2
payloads. Instead, it fail-closes on fresh `/depth/min_front_m`, `/vo/health`,
and `/savo_perception/obstacle_cloud/health` signals when their producers are
enabled. VO and obstacle-cloud requirements follow the canonical Edge feature
flags, so disabled optional consumers do not cause false failures.

The separate raw topic monitor remains available for explicit diagnostics, but
is deliberately excluded from canonical production bringup because it consumes
the high-bandwidth camera streams:

```bash
ros2 run savo_realsense camera_topic_monitor_node --ros-args \
  --params-file install/savo_realsense/share/savo_realsense/config/realsense_d435_nodes.yaml
```

The standalone VO-oriented launch retains its explicit relay switch:

```bash
ros2 launch savo_realsense realsense_vo.launch.py \
  enable_observer_color_relay:=true
```

The canonical driver configuration is `realsense_d435_camera.yaml`; it already
enables the local raw pointcloud required by the obstacle filter. The legacy
`realsense_pointcloud_camera.yaml` and `realsense_pointcloud_nodes.yaml` files
remain installed for their standalone launch interface and are contract-tested
to match the canonical production configuration.

The diagnostic topic monitor records `expected_pointcloud_hz` as metadata. The
production health node keeps the established `/realsense/status` fields, but
projects them from lightweight downstream proof: depth fields come from the
front-depth signal, color/aligned-depth fields come from VO health, and the
pointcloud field comes from obstacle-cloud health. Any required signal that is
missing, stale, or unhealthy fails closed under the existing stale timeout.

Edge hardware validation with the D435 and RealSense ROS 4.58.1 established
`pointcloud__neon_.stream_filter: 1` as the reliable depth-stream selector for
publishing `/camera/camera/depth/color/points`. Selector `0` failed on this
runtime with `No matching stream for texture 'Process - Any'`. Production uses
the hardware-validated selector `1`; any missing, stale, malformed, or
transform-failing cloud remains a real failure.

Recommended writing order
Step 1 — package metadata

Start with:

package.xml
setup.py
setup.cfg
CMakeLists.txt
resource/savo_realsense
savo_realsense/__init__.py
savo_realsense/version.py
savo_realsense/constants.py

Purpose: make the package buildable.

Do this first before writing nodes.

Step 2 — ROS contracts

Then write:

savo_realsense/ros/topic_contract.py
savo_realsense/ros/frame_contract.py
savo_realsense/ros/qos_profiles.py
savo_realsense/ros/params.py

This matches your savo_base style.

Why first? Because all nodes and tests should use the same topic/frame names.

Production rule:

No hardcoded camera topic names randomly inside nodes.
Step 3 — utilities

Then write:

savo_realsense/utils/topic_names.py
savo_realsense/utils/frame_names.py
savo_realsense/utils/timing.py
savo_realsense/utils/camera_checks.py
savo_realsense/utils/depth_image.py
savo_realsense/utils/param_loader.py
savo_realsense/utils/diagnostics.py

This gives us reusable logic before nodes.

Same philosophy as savo_base/utils.

Step 4 — models

Then write:

savo_realsense/models/camera_status.py
savo_realsense/models/stream_status.py
savo_realsense/models/depth_sample.py

Keep them small.

They should not depend heavily on ROS. They are internal data containers.

Step 5 — config files

Then write:

config/realsense_minimal.yaml
config/realsense_d435.yaml
config/realsense_vo_profile.yaml
config/realsense_nav_profile.yaml
config/camera_frames.yaml
config/qos.yaml

Recommended order:

1. realsense_minimal.yaml
2. realsense_d435.yaml
3. camera_frames.yaml
4. qos.yaml
5. vo/nav profiles later

Because minimal launch is the first hardware test.

Step 6 — launch files

Then write:

launch/realsense_minimal.launch.py
launch/realsense_bringup.launch.py
launch/realsense_vo.launch.py
launch/realsense_diagnostics.launch.py

Recommended order:

1. realsense_minimal.launch.py
2. realsense_bringup.launch.py
3. diagnostics launch
4. VO profile launch later
Step 7 — nodes

Then write nodes one by one:

camera_topic_monitor_node.py
camera_health_node.py
depth_front_min_node.py

Recommended order:

First node
camera_topic_monitor_node.py

Because it is simple and immediately useful.

It checks:

color topic alive
depth topic alive
camera info alive
pointcloud alive if enabled
topic rate
stale timeout
Second node
camera_health_node.py

The production C++ node combines lightweight depth, VO, and obstacle-cloud
signals into one camera health status without copying camera payloads. The
Python node remains a fallback/diagnostic implementation.

Third node
depth_front_min_node.py

But important: we must decide this carefully.

My expert preference:

depth_front_min_node.py should probably stay in shared/savo_perception

Because depth front-min is not only a RealSense ownership function. It is part of perception/safety processing.

So for savo_realsense, I would either:

Option A: keep depth_front_min_node.py empty for now
Option B: remove it from first implementation
Option C: use it only as an edge-side helper, but final safety fusion still stays in savo_perception

For production, I recommend:

Do not implement depth_front_min_node.py inside savo_realsense first.
Implement camera ownership first.
Step 8 — diagnostics

Then write:

realsense_topic_check.py
realsense_frame_check.py
realsense_usb_check.py
report_formatter.py

This mirrors savo_base/diagnostics.

Diagnostics should help you answer:

Is RealSense detected by USB?
Are camera topics publishing?
Are topic rates stable?
Are expected frames present?
Step 9 — scripts

Then write:

scripts/realsense_smoke_test_cli.py
scripts/dump_effective_realsense_params.py

Same style as savo_base/scripts.

These are very good for real robot testing.

Step 10 — tests

Write tests gradually:

test_imports.py
test_topic_names.py
test_frame_names.py
test_depth_image.py
test_camera_checks.py
test_param_loader.py

Start with:

test_imports.py
test_topic_names.py
test_frame_names.py

Then add depth and camera check tests after utilities are written.
