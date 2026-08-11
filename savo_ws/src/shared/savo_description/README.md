# savo_description

Shared Robot Savo description package.

This package owns the physical robot model, URDF/Xacro files, fixed TF frames, sensor mounting frames, wheel geometry, RViz views, and description validation tools.

## Role

`savo_description` is shared by both:

- `savo-core`
- `savo-edge`

It provides the common robot frame model for:

- base and chassis
- 4 mecanum wheels
- LiDAR
- IMU
- RealSense/depth camera
- ToF sensors
- ultrasonic sensor
- display
- ReSpeaker
- two-Pi mounting frames
- Nav2 / 3D costmap frame alignment

## Important rule

This package publishes only fixed robot frames.

It does not publish:

- `map -> odom`
- `odom -> base_footprint`

Those are owned by SLAM, AMCL, EKF, or navigation.

## Geometry profile contract

The package is installed as a normal `ament_cmake` runtime package. Its launch,
URDF/Xacro, configuration, RViz, mesh, and helper-script assets are available
from `share/savo_description` after a workspace build.

The revision-2 canonical profile contains measured plate, wheel-center, fixed
sensor, and neutral head translations:

- `config/profiles/robot_savo_core_v1.yaml`

The files below are human-readable mirrors guarded by contract tests:

- `config/robot_dimensions.yaml`
- `config/wheel_geometry.yaml`
- `config/sensor_mounts.yaml`

The profile remains provisional. IMU/LiDAR orientation, D435 internal
extrinsics, head servo signs, plate Z datum, wheel width/mass/inertials, and
remaining component geometry still block physical lock.

`base_link` uses the reviewed wheel axle-plane convention at `+0.0325 m` from
`base_footprint`. The generated `nav2_footprint.yaml` is the measured plate
envelope only; production Nav2 retains a larger conservative footprint.

The LiDAR frame contract is:

```text
base_link -> laser_frame
```

`/scan` publishers must use `laser_frame` in `LaserScan.header.frame_id`.
