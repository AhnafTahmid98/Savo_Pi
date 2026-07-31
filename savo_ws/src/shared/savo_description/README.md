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

## AM-0 description contract

The package is installed as a normal `ament_cmake` runtime package. Its launch,
URDF/Xacro, configuration, RViz, mesh, and helper-script assets are available
from `share/savo_description` after a workspace build.

The files below document the values already encoded in the current Xacro model:

- `config/robot_dimensions.yaml`
- `config/wheel_geometry.yaml`
- `config/sensor_mounts.yaml`

They are marked `final_physical_measurement_required: true`. They preserve the
current source baseline and prevent silent Xacro/config drift; they do not claim
that the final Nav2 footprint or sensor offsets have been physically locked.

The LiDAR frame contract is:

```text
base_link -> laser_frame
```

`/scan` publishers must use `laser_frame` in `LaserScan.header.frame_id`.
