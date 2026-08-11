# TF Frame Authority

Every transform has one runtime owner.

| Transform subtree | Authority | State |
| --- | --- | --- |
| `map -> odom` | SLAM during mapping or AMCL during saved-map navigation | Dynamic, mutually exclusive |
| `odom -> base_footprint` | `robot_localization` EKF | Dynamic |
| `base_footprint -> base_link` | `robot_state_publisher` | Fixed +0.0325 m axle-plane convention |
| `base_link -> chassis/plates/wheels/fixed sensors` | `robot_state_publisher` | Fixed, profile revision 2 |
| `base_link -> camera_link -> camera_color/depth_*` | `robot_state_publisher` | Fixed; RealSense driver TF disabled |
| `base_link -> pantilt_mount_link` | `robot_state_publisher` | Fixed measured translation |
| `pantilt_mount_link -> pantilt_pan_link -> pantilt_tilt_link -> pi_camera_link -> pi_camera_optical_frame` | `savo_head/head_tf_node` | Dynamic, `publish_tf=false`, calibration false |

The RealSense driver uses `publish_tf=false` in every production profile.
Description-owned color/depth frame translations are still provisional zeros;
they must not be described as calibrated internal D435 extrinsics. Do not enable
driver TF without redesigning and regression-testing the entire subtree.

Head translations are measured, but unknown `pan_sign`/`tilt_sign` keep the
dynamic publisher fail-closed. `savo_head` must never publish
`base_link -> pantilt_mount_link`.

Wheel odometry publishes messages with `publish_tf=false` and cannot compete
with the EKF. Mapping and saved-map navigation must prevent simultaneous SLAM
and AMCL ownership of `map -> odom`.
