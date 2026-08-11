# Robot SAVO TF authority

| Parent | Child | Authority |
| --- | --- | --- |
| `map` | `odom` | SLAM Toolbox or localization, never both |
| `odom` | `base_footprint` | `savo_localization` EKF |
| `base_footprint` | `base_link` | `robot_state_publisher` |
| `base_link` | fixed sensor and interface links | `robot_state_publisher` / `savo_description` |
| `base_link` | `pantilt_mount_link` | `robot_state_publisher` / `savo_description` |
| `pantilt_mount_link` | `pantilt_pan_link` | `savo_head/head_tf_node` |
| `pantilt_pan_link` | `pantilt_tilt_link` | `savo_head/head_tf_node` |
| `pantilt_tilt_link` | `pi_camera_link` | `savo_head/head_tf_node` |
| `pi_camera_link` | `pi_camera_optical_frame` | `savo_head/head_tf_node` |

The RealSense driver must use `publish_tf: false`; its fixed model frames are owned by `robot_state_publisher`. The internal color/depth translations are provisional zeros, not calibrated D435 extrinsics. `savo_head` must not publish `base_link -> pantilt_mount_link`.

Measured head translations do not bypass calibration gating. `savo_head` keeps
`publish_tf: false` and `transforms_calibrated: false` until pan/tilt direction
and runtime image/TF behavior are physically validated.
