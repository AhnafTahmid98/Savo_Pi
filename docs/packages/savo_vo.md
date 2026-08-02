# savo_vo

`savo_vo` runs on `savo-edge` and owns RGB-D visual odometry derived from the
RealSense streams. Its production C++ path contains:

- `rgbd_odometry_node`;
- `vo_republisher_node`;
- `vo_health_node`;
- `vo_diagnostics_node`.

Canonical outputs include `/vo/odom`, `/vo/odom/raw`, `/vo/status`,
`/vo/health`, and `/vo/tracking_quality`. Core localization may fuse VO only
when the configured bringup profile enables it and VO health/freshness pass.
VO does not own `map→odom` and does not replace wheel odometry or the IMU.

## Validation

Use `docs/testing/vo_test_plan.md`. Before physical motion, verify RealSense
stream freshness, camera calibration, frame IDs, timestamp behavior, covariance,
and controlled agreement with wheel/IMU localization. Loss or stale VO must
degrade safely rather than block STOP or become a false pose authority.
