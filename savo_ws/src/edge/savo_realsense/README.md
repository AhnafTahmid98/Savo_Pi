# Robot Savo RealSense

## Optional observer color relay

The production D435 color stream remains full-rate `640x480x30` at
`/camera/camera/color/image_raw` on Edge. For the external observer only,
`realsense_vo.launch.py` accepts
`enable_observer_color_relay:=true` to start one standard `image_transport`
republisher on Edge:

```text
/camera/camera/color/image_raw
  -> JPEG compression on Edge
  -> /savo_observer/d435/color/image_raw/compressed
  -> Ubuntu RViz compressed transport
```

Production depth at `/camera/camera/depth/image_rect_raw` and the raw cloud at
`/camera/camera/depth/color/points` also remain unchanged and available.

The relay defaults to disabled because large raw DDS image samples are only a
problem on the Edge Wi-Fi to bridged Ubuntu VM observer path. It does not
change camera FPS or resolution, the raw color topic, raw pointcloud,
navigation or perception semantics, VO, or fixed D435 TF ownership. The
separate RealSense startup firmware notification is not suppressed or
reinterpreted by this relay.

Normal production remains:

```bash
ros2 launch savo_realsense realsense_vo.launch.py
```

Enable the observer JPEG relay explicitly on Edge with:

```bash
ros2 launch savo_realsense realsense_vo.launch.py \
  enable_observer_color_relay:=true
```

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

It combines stream status into one camera health status.

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
