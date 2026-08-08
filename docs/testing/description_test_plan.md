# Robot Description and Geometry Test Plan

## Objective

Validate the Robot Savo physical model, fixed TF contract, dynamic-head boundary, Nav2 footprint, coordinate conventions, and geometry-lock process before any motion-capable production mode is authorized.

The current profile `robot_savo_core_v1.yaml` is marked `provisional`. That state is an intentional motion blocker until a physical measurement pass is completed, reviewed, and committed.

## Ownership boundary

`savo_description` owns:

- URDF/Xacro;
- fixed robot frames;
- chassis, wheel, and sensor geometry;
- navigation footprint generation;
- RViz model assets;
- geometry validation tools.
It does not own:

- `map -> odom`;
- `odom -> base_footprint`;
- dynamic head pan/tilt transforms;
- live RealSense internal transforms;
- sensor data timestamps.
Dynamic head TF is owned by `savo_head`.

## Required equipment

- robot on a level surface;
- calibrated ruler/tape and calipers where appropriate;
- angle gauge or square;
- scale for mass values when mass is used;
- camera for measurement evidence;
- Core/observer ROS 2 Jazzy environment;
- RViz;
- approved measurement worksheet.

## Coordinate convention

The production convention is:

```text
+X forward
+Y left
+Z up
+positive yaw counter-clockwise when viewed from above

```

Every measurement, mount pose, wheel sign, sensor frame, and footprint point must use this convention.

## Stage D0 — package build and tests

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_description --symlink-install
source install/setup.bash
colcon test --packages-select savo_description --ctest-args --output-on-failure
colcon test-result --verbose

```

Pass criteria: zero build/test failures and all installed description assets present.

## Stage D1 — provisional profile inspection

```bash
cd ~/Savo_Pi/savo_ws/src/shared/savo_description
python3 scripts/validate_geometry_profile.py \
  config/profiles/robot_savo_core_v1.yaml \
  --allow-provisional
python3 scripts/print_geometry_summary.py \
  config/profiles/robot_savo_core_v1.yaml

```

Expected current result:

- profile validates only when provisional state is allowed;
- summary states `Physical measurement lock: NOT COMPLETE`;
- a canonical SHA-256 digest and derived Nav2 footprint are printed.
Negative gate:

```bash
python3 scripts/validate_geometry_profile.py \
  config/profiles/robot_savo_core_v1.yaml \
  --require-locked

```

While the profile remains provisional, this command must fail. If it passes unexpectedly, stop and investigate the validator/profile.

## Stage D2 — generate and check URDF

After building and sourcing:

```bash
bash ~/Savo_Pi/savo_ws/src/shared/savo_description/scripts/generate_urdf.sh \
  /tmp/robot_savo.urdf
bash ~/Savo_Pi/savo_ws/src/shared/savo_description/scripts/check_urdf.sh

```

Verify:

- Xacro expands without error;
- URDF parser reports a valid tree;
- every frame/joint name is unique;
- no unresolved substitution remains;
- no zero-length or impossible geometry appears unintentionally.
Retain `/tmp/robot_savo.urdf` with the test evidence.

## Stage D3 — physical measurement pass

Power the robot down and make it mechanically safe. Measure and record at least:

### Chassis

- overall length, width, and height;
- `base_footprint` to `base_link` vertical offset;
- deck thickness and spacing;
- mass and center-of-mass observations when available.

### Wheels

- loaded wheel radius;
- wheel width;
- front/rear X locations;
- left/right Y locations;
- axle alignment;
- mecanum roller orientation;
- clearance at full load.

### Sensors and devices

For each mount, record parent frame, X/Y/Z, roll/pitch/yaw, measurement reference, and uncertainty:

- RPLIDAR;
- BNO055;
- RealSense D435;
- left/right ToF;
- front ultrasonic;
- display;
- ReSpeaker;
- pan/tilt mount;
- Pi Camera optical orientation.
Take photographs that show the measurement origin and direction.

## Stage D4 — update and review profile

Update only measured fields in:

```text
savo_ws/src/shared/savo_description/config/profiles/robot_savo_core_v1.yaml

```

Fill metadata:

- measurement state;
- measured by;
- measurement date;
- source/worksheet identity;
- incremented geometry revision;
- notes and uncertainties.
A reviewer must independently check sign conventions, left/right symmetry assumptions, units, and sensor optical-frame rotations.

Do not change `measurement_state` to `locked` until the review is complete.

## Stage D5 — locked profile gate

After approval:

```bash
python3 scripts/validate_geometry_profile.py \
  config/profiles/robot_savo_core_v1.yaml \
  --require-locked
python3 scripts/print_geometry_summary.py \
  config/profiles/robot_savo_core_v1.yaml

```

Record the new canonical digest and footprint. The digest becomes release-controlled data used by map/navigation release verification.

## Stage D6 — TF tree validation

Launch the description in the supported bringup and inspect TF:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf_static --once

```

Use the package helper where appropriate:

```bash
bash ~/Savo_Pi/savo_ws/src/shared/savo_description/scripts/print_tf_tree.sh

```

Verify:

- one connected robot tree;
- no duplicate fixed-transform authority;
- `base_footprint -> base_link` is correct;
- wheel frames are in the correct quadrants;
- `base_link -> laser_frame` matches the LiDAR mount;
- IMU and range frames match physical orientation;
- RealSense and optical frames follow ROS optical conventions;
- head pan/tilt frames are absent from fixed authority when dynamic ownership is active;
- no description publisher creates `map -> odom` or `odom -> base_footprint`.

## Stage D7 — RViz visual inspection

Open the model:

```bash
bash ~/Savo_Pi/savo_ws/src/shared/savo_description/scripts/open_rviz_model.sh

```

Inspect from front, rear, left, right, top, and isometric views. Compare with photographs.

Check:

- wheel locations and roller orientation;
- sensor heights and left/right placement;
- camera optical axes;
- head movement frames;
- chassis collision geometry;
- frame labels;
- absence of mirrored axes.

## Stage D8 — footprint validation

Use the printed derived footprint and compare it with the physical maximum envelope, including wheels and fixed protrusions. Confirm configured padding is appropriate for the current operating environment.

In RViz/Nav2:

- display robot footprint;
- rotate the robot model through 360 degrees;
- verify footprint encloses the actual body;
- verify no fixed component extends beyond the footprint unnoticed;
- verify padding does not make narrow passages falsely impossible without review.
A footprint that is too small is a safety failure. A footprint that is overly large is an operational/tuning issue and must still be reviewed.

## Stage D9 — integration with live frames

With Core and Edge in safe idle, validate:

- LiDAR scan header is `laser_frame`;
- IMU frame matches `imu_link`;
- RealSense topics use the approved camera frames;
- odometry/localization publishes only the dynamic frames it owns;
- head movement updates the dynamic camera chain without changing fixed mounts;
- all timestamps are current and TF lookup succeeds at sensor times.

## Change-triggered revalidation

Repeat the applicable stages after:

- moving any sensor or computer;
- changing wheels, tires, load, or chassis;
- modifying pan/tilt hardware;
- changing URDF/Xacro or geometry profile code;
- changing Nav2 footprint padding;
- changing camera self-filter bounds;
- releasing maps created under a different geometry digest.

## Abort/fail criteria

Fail the description gate if:

- profile state or metadata is inaccurate;
- locked validation passes without physical review;
- URDF is invalid;
- frame names/authority conflict;
- measured signs or units are ambiguous;
- footprint does not contain the robot;
- TF tree is disconnected or duplicated;
- live sensor frame IDs disagree with the description;
- map/location release geometry digest is inconsistent.

## Evidence

Retain:

- measurement worksheet and photographs;
- reviewed profile diff;
- profile digest and revision;
- generated URDF;
- `check_urdf` output;
- TF graph;
- RViz screenshots;
- derived footprint and physical-envelope comparison;
- reviewer and date;
- pass/fail decision.

## Acceptance

The description is accepted for motion only when the profile is physically measured, independently reviewed, marked locked, validates with `--require-locked`, produces a correct URDF/TF tree, and yields a footprint that safely encloses the real robot.
