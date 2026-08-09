# Localization Architecture

Core owns the authoritative local odometry chain; Edge can contribute optional visual odometry.

## Data and TF flow

```text
four GPIO encoders -> /wheel/odometry --+
BNO055 IMU --------> /imu/data ---------+-> robot_localization EKF
optional Edge VO --> /vo/odom ----------+        |
                                                  +-> /odometry/filtered
                                                  +-> odom -> base_footprint

SLAM (mapping) or AMCL (saved map) -> map -> odom
robot_state_publisher              -> base_footprint -> fixed robot frames
```

Wheel odometry is configured for `30 Hz`, `0.065 m` wheel diameter, `0.165 m` wheelbase, `0.165 m` track, and `0.5 s` encoder timeout. Encoder GPIO pairs are FL `20/21`, FR `13/25`, RL `23/24`, and RR `12/26`; configured CPR is 20 with x4 decoding. The BNO055 uses Core I2C bus 1 at `0x28`, NDOF mode, `25 Hz`.

The EKF runs at `30 Hz`, uses a `0.2 s` sensor timeout, operates in 2D, publishes `/odometry/filtered`, and is the sole `odom -> base_footprint` TF authority. Wheel odometry has `publish_tf=false`. VO fusion is `false` by default; when enabled, `/vo/odom` must use the configured `odom` and `base_footprint` frames and validated covariance/timestamps.

## Mode ownership

Live mapping starts SLAM as `map -> odom` owner. Saved-map navigation starts AMCL as that owner. They must never publish the same transform concurrently. The description owns fixed `base_footprint -> base_link -> sensor/wheel` transforms.

## Known geometry blocker

The description places wheel centers at x `+/-0.115 m` and y `+/-0.100 m`, implying `0.230 m` longitudinal separation and `0.200 m` lateral separation, while base/localization kinematics are configured as `0.165/0.165 m`. All values are provisional. The discrepancy must be measured and reconciled before locking geometry or tuning odometry.

Source tests validate frames, configuration, and health contracts. Hardware work remains for encoder pin/polarity/counts, wheel effective radius, track/wheelbase, IMU axis/calibration, covariance, EKF drift, VO scale/latency, TF uniqueness, and transitions between SLAM and AMCL.
