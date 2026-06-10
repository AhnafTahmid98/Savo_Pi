# savo_vo

`savo_vo` is the visual odometry package for Robot Savo.

It runs on `savo-edge`, consumes RealSense RGB-D camera topics, estimates visual motion, and publishes visual odometry for later fusion in `savo_localization`.

## Role

`savo_vo` does not own the camera.

`savo_realsense` owns RealSense bringup, camera topics, and camera health monitoring.

`savo_vo` subscribes to those topics and publishes:

```text
/vo/odom
/vo/status
/vo/health
/vo/tracking_quality
```

Later, `savo_localization` can fuse `/vo/odom` with wheel odometry and IMU.

## Package Boundary

```text
savo_realsense
    RealSense camera bringup, RGB/depth topics, camera health

savo_vo
    RGB-D visual odometry, VO quality checks, /vo/odom output

savo_localization
    EKF fusion of wheel odometry, IMU, and VO

savo_nav
    Nav2 navigation using fused localization

savo_control
    command shaping, muxing, recovery behavior

savo_base
    safe motor command execution on real hardware
```

## Recommended Writing Order

### Step 1 — Package Metadata

Start with:

```text
package.xml
setup.py
setup.cfg
CMakeLists.txt
resource/savo_vo
savo_vo/__init__.py
savo_vo/version.py
```

Purpose: make the package buildable before writing runtime logic.

Do this first so `colcon` can identify and install the package correctly.

### Step 2 — Contracts

Then write:

```text
savo_vo/contracts/topic_names.py
savo_vo/contracts/frame_names.py
savo_vo/contracts/parameter_names.py
```

Why first? VO nodes, launch files, configs, and tests should use the same topic, frame, and parameter names.

Production rule:

```text
Do not hardcode VO topic names randomly inside nodes.
```

### Step 3 — Pure Models

Then write:

```text
savo_vo/models/vo_status.py
savo_vo/models/tracking_report.py
savo_vo/models/odometry_quality.py
savo_vo/models/vo_sample.py
```

Keep these files small.

They should not import ROS.

They are internal data containers used by the estimator, health logic, and adapters.

### Step 4 — Pure Core Logic

Then write:

```text
savo_vo/core/timestamp_sync.py
savo_vo/core/tracking_quality.py
savo_vo/core/motion_quality.py
savo_vo/core/odom_checks.py
savo_vo/core/covariance_builder.py
savo_vo/core/vo_state_machine.py
```

Purpose: keep VO validation and quality logic testable without ROS.

This is where we check:

```text
timestamp sync
tracking quality
motion jumps
odometry publish safety
covariance selection
VO state selection
```

Production rule:

```text
Core logic must run with normal Python tests.
```

### Step 5 — Utilities

Then write:

```text
savo_vo/utils/numeric.py
savo_vo/utils/geometry.py
savo_vo/utils/validation.py
savo_vo/utils/ros_time.py
```

Keep utilities small.

Do not put ROS QoS profiles here.

Even `ros_time.py` should avoid importing `rclpy` directly so normal Python tests stay clean.

### Step 6 — ROS-Specific Helpers

Then write:

```text
savo_vo/ros/qos.py
```

This file is ROS-specific because it imports `rclpy` QoS classes.

Use `BEST_EFFORT` for RealSense image/depth input topics.

Use `RELIABLE` for VO odometry, status, health, and diagnostics.

Production rule:

```text
Do not import savo_vo.ros.qos inside package-level __init__.py files.
```

### Step 7 — Adapters

Then write:

```text
savo_vo/adapters/camera_info_adapter.py
savo_vo/adapters/image_adapter.py
savo_vo/adapters/odometry_adapter.py
savo_vo/adapters/diagnostics_adapter.py
savo_vo/adapters/realsense_topic_adapter.py
```

Adapters convert between ROS-facing data and internal models.

Examples:

```text
CameraInfo -> CameraIntrinsics
Image -> OpenCV/numpy array
VOSample -> nav_msgs/Odometry
VOStatus -> DiagnosticStatus
```

Production rule:

```text
Adapters may import ROS packages, but core/models/contracts should not.
```

### Step 8 — Estimators

Then write:

```text
savo_vo/estimators/base_estimator.py
savo_vo/estimators/feature_tracker.py
savo_vo/estimators/pose_estimator.py
savo_vo/estimators/scale_guard.py
savo_vo/estimators/rgbd_vo_estimator.py
```

Recommended order:

```text
1. base_estimator.py
2. feature_tracker.py
3. pose_estimator.py
4. scale_guard.py
5. rgbd_vo_estimator.py
```

Purpose: build the VO pipeline behind a clean estimator interface.

The first estimator uses RGB-D input:

```text
RealSense color image
RealSense depth image
CameraInfo calibration
feature tracking
PnP pose estimation
scale/motion guards
VOSample output
```

Real hardware testing will confirm camera axis mapping and tuning values.

### Step 9 — Config Files

Then write:

```text
config/rgbd_odometry.yaml
config/vo_health.yaml
config/vo_covariance.yaml
config/qos.yaml
config/frames.yaml
```

Recommended order:

```text
1. rgbd_odometry.yaml
2. vo_health.yaml
3. vo_covariance.yaml
4. qos.yaml
5. frames.yaml
```

The config should match the RealSense VO topics published by `savo_realsense`.

Default input topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

Default output topics:

```text
/vo/odom
/vo/status
/vo/health
/vo/tracking_quality
```

Keep this default:

```yaml
publish_tf: false
```

The EKF in `savo_localization` should own the final `odom -> base_link` transform.

### Step 10 — Launch Files

Then write:

```text
launch/rgbd_odometry.launch.py
launch/vo_health.launch.py
launch/vo_diagnostics.launch.py
launch/vo_bringup.launch.py
```

Recommended order:

```text
1. rgbd_odometry.launch.py
2. vo_health.launch.py
3. vo_diagnostics.launch.py
4. vo_bringup.launch.py
```

First test should start RealSense and VO separately.

Terminal 1:

```bash
ros2 launch savo_realsense realsense_vo.launch.py
```

Terminal 2:

```bash
ros2 launch savo_vo rgbd_odometry.launch.py
```

Later, `vo_bringup.launch.py` can optionally include the RealSense launch file with:

```bash
use_realsense:=true
```

Do not add a hard package dependency on `savo_realsense` until that becomes the official production bringup path.

### Step 11 — Nodes

Then write nodes one by one:

```text
savo_vo/nodes/rgbd_odometry_node.py
savo_vo/nodes/vo_health_node.py
savo_vo/nodes/vo_diagnostics_node.py
savo_vo/nodes/vo_republisher_node.py
```

Recommended order:

```text
1. rgbd_odometry_node.py
2. vo_health_node.py
3. vo_diagnostics_node.py
4. vo_republisher_node.py
```

`rgbd_odometry_node.py` is the main runtime node.

It subscribes to:

```text
color image
depth image
color camera info
```

It publishes:

```text
/vo/odom
/vo/status
/vo/health
/vo/tracking_quality
```

`vo_health_node.py` checks whether `/vo/odom` is alive and reports stale/waiting/ok status.

`vo_diagnostics_node.py` converts VO health into standard ROS diagnostics.

`vo_republisher_node.py` is optional. It is useful later if another backend publishes `/vo/odom/raw` and we want to normalize frames or topic names.

### Step 12 — Tests

Write tests gradually:

```text
test_imports.py
test_topic_names.py
test_frame_names.py
test_parameter_names.py
test_vo_status.py
test_tracking_report.py
test_timestamp_sync.py
test_tracking_quality.py
test_motion_quality.py
test_odom_checks.py
test_covariance_builder.py
test_geometry.py
```

Start with:

```text
test_imports.py
test_topic_names.py
test_frame_names.py
test_parameter_names.py
```

Then test models and core logic.

Normal Python tests should run with:

```bash
python3 -m pytest test -q
```

Production rule:

```text
Normal Python tests should not require a sourced ROS environment.
```

## Real Hardware Validation Later

After PC build and tests pass, move to `savo-edge` for RealSense testing.

Real hardware test order:

```text
1. Build and source workspace
2. Start RealSense VO profile
3. Confirm camera topics
4. Start RGB-D VO node
5. Confirm /vo/status
6. Confirm /vo/tracking_quality
7. Confirm /vo/odom
8. View /vo/odom in RViz
9. Record rosbag for debugging
10. Tune thresholds and axis mapping if needed
```

Do not fuse `/vo/odom` into EKF until standalone VO output is stable.

## First PC Checks

From workspace root:

```bash
cd ~/Savo_Pi/savo_ws
colcon build --packages-select savo_vo
```

Then source:

```bash
source install/setup.bash
```

Check executables:

```bash
ros2 pkg executables savo_vo
```

Expected executables:

```text
savo_vo rgbd_odometry_node
savo_vo vo_health_node
savo_vo vo_diagnostics_node
savo_vo vo_republisher_node
```

## First Runtime Test Later

Terminal 1:

```bash
ros2 launch savo_realsense realsense_vo.launch.py
```

Terminal 2:

```bash
ros2 launch savo_vo rgbd_odometry.launch.py
```

Check output:

```bash
ros2 topic echo /vo/status
ros2 topic echo /vo/health
ros2 topic echo /vo/tracking_quality
ros2 topic echo /vo/odom
```

## RViz

Start RViz:

```bash
rviz2
```

Set fixed frame:

```text
odom
```

Add display:

```text
Odometry
```

Set topic:

```text
/vo/odom
```

Later we may add `/vo/path` for easier visual debugging.

## Current Production Decision

Keep `savo_vo` connected to `savo_realsense` through ROS topics.

Do not import `savo_realsense` Python code inside `savo_vo`.

Do not make `savo_vo` publish final TF by default.

Do not fuse VO into EKF until standalone `/vo/odom` is stable on real hardware.
