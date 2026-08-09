# TF Frame Authority

Every transform has exactly one runtime owner. Frame names are configured centrally by `savo_description`; changing a name or mount is an interface and safety change.

| Transform subtree | Authority | State |
| --- | --- | --- |
| `map -> odom` | SLAM during mapping, AMCL during saved-map navigation | Dynamic, mutually exclusive |
| `odom -> base_footprint` | `robot_localization` EKF | Dynamic |
| `base_footprint -> base_link` | Core robot state publisher | Fixed, provisional geometry |
| `base_link -> chassis/decks/wheels/sensors/Pi mounts` | Core robot state publisher from URDF | Fixed, provisional geometry |
| `camera_link -> camera_color/depth_*` | Robot state publisher | Fixed; RealSense `publish_tf=false` |
| `base_link -> depth_obstacle_frame` | Robot state publisher | Fixed, provisional |
| `pantilt_mount_link -> pan -> tilt -> pi_camera` | `savo_head` head TF node | Dynamic, currently `publish_tf=false`, calibration false |

The wheel odometry node publishes odometry messages with `publish_tf=false`; it must not compete with the EKF. Edge VO publishes odometry data, not the authoritative Core TF. RealSense driver TF is disabled to avoid duplication. Mapping and saved-map navigation launch must prevent SLAM/AMCL coexistence as `map -> odom` owner.

Configured fixed sensor translations include LiDAR `[0,0,0.205]`, IMU `[0,0,0.075]`, D435 `[0.135,0,0.155]`, ToF left/right `[0.125,+/-0.090,0.070]`, and ultrasonic `[0.150,0,0.070]` metres from `base_link`, with zero RPY. These are source-configured provisional values, not measurements.

TF failure is fail-closed for localization, mapping, navigation, obstacle projection, and semantic confirmation. Validate generated URDF, duplicate frames, timestamp freshness, `map -> base_footprint` continuity, optical conventions, physical sensor axes, and mode transitions. Lock the geometry profile and calibrate the head chain before enabling production motion or dynamic head TF.
