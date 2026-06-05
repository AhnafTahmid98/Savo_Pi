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
