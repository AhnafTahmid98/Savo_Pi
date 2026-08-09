# ROS 2 Topic Contracts

This registry lists cross-subsystem topics that form architectural contracts. Package-local diagnostics and implementation details remain in package documentation and source.

## Motion and safety

| Topic | Type | Producer | Consumer / rule |
| --- | --- | --- | --- |
| `/cmd_vel_manual`, `/cmd_vel_auto`, `/cmd_vel_nav`, `/cmd_vel_recovery` | `geometry_msgs/Twist` | Approved lane owner | `savo_control`; never base |
| `/cmd_vel_mux` | `geometry_msgs/Twist` | Control mux | Control shaper |
| `/cmd_vel` | `geometry_msgs/Twist` | `savo_control` | Perception gate only |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | `savo_perception` | `savo_base` only |
| `/safety/stop` | `std_msgs/Bool` | Core perception | Gate, control/readiness observers; true fails closed |
| `/safety/slowdown_factor` | `std_msgs/Float32` | Core perception | Command gate |
| `/savo_perception/safety_state` | `std_msgs/String` | Core perception | Readiness/observers |

## Localization, scan, and TF

| Topic/transform | Producer | Contract |
| --- | --- | --- |
| `/scan` | `savo_lidar` | `sensor_msgs/LaserScan`, `laser_frame` |
| `/imu/data` | `savo_localization` | `sensor_msgs/Imu`, `imu_link` |
| `/wheel/odometry` | `savo_localization` | `nav_msgs/Odometry`, `odom -> base_footprint`, no TF |
| `/vo/odom` | Edge `savo_vo` | Optional `nav_msgs/Odometry`; fusion off by default |
| `/odometry/filtered` | EKF | Authoritative local odometry |
| `odom -> base_footprint` | EKF | Exactly one publisher |
| `map -> odom` | SLAM or AMCL | Mode-exclusive publisher |
| `/tf_static` | robot state publisher | URDF fixed joints |

## Perception and Edge

| Topic | Producer | Use |
| --- | --- | --- |
| `/camera/camera/depth/color/points` | RealSense | Edge cloud-filter input |
| `/savo_perception/obstacles/points` | Edge perception | Optional Nav2 voxel input |
| `/depth/min_front_m` | `savo_realsense` | Front-depth observation, not Core stop replacement |
| `/savo_perception/range/left_m`, `/right_m`, `/front_ultrasonic_m` | Core perception | Range observations |
| `/realsense/status`, `/vo/health` | Edge producers | Edge/Core readiness and diagnostics |

## Distributed state

Core and Edge readiness publish retained state and Boolean ready topics plus reliable heartbeat/diagnostics under `/savo_bringup/core/*` and `/savo_bringup/edge/*`. Supervisor, base, control, localization, mapping, navigation, power, bridge, speech, and UI publish package-owned state/health topics documented in their central pages.

Retained state generally uses reliable transient-local QoS; heartbeats use reliable streaming QoS; sensors use the driver-appropriate sensor profile. Consumers must still enforce freshness—durability is not liveness.

Actions and services are not topics. Their generated definitions in `savo_msgs` and package adapters are authoritative; do not infer them from similarly named state topics. Validate runtime types, QoS compatibility, publishers, rates, timestamps, and single-authority rules with `ros2 topic info -v`, `ros2 interface show`, TF tools, and package health outputs.
