# ROS 2 Topic Contracts

Authoritative topic names, message types, QoS profiles, frame IDs, and publisher/subscriber ownership for Robot Savo.

Topic names are defined in code (`constants.py`, `topic_contract.py`, `topic_names.py`) inside each package. This document is a cross-package reference, not the source of truth.

---

## Visual odometry topics

Published by `savo_vo` on `savo-edge`. Consumed by `savo_localization` EKF on `savo-core`.

| Topic          | Message type                      | Publisher                 | Subscribers               | QoS         |
| -------------- | --------------------------------- | ------------------------- | ------------------------- | ----------- |
| `/vo/odom`     | `nav_msgs/Odometry`               | `savo_vo`                 | `savo_localization` (EKF) | Reliable    |
| `/vo/status`   | `savo_msgs/VoStatus`              | `savo_vo`                 | Dashboard, diagnostics    | Best effort |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `savo_vo` (contributing)  | `/diagnostics_agg`        | Best effort |

### /vo/odom contract

```text
header.frame_id:   odom
child_frame_id:    camera_link
rate:              10–15 Hz
covariance:        non-zero (6×6 pose + 6×6 twist)
```

The `odom → camera_link` transform described by this message represents the VO estimate of camera motion in the fixed odometry frame. The EKF uses this to supplement wheel + IMU odometry.

---

## RealSense camera topics

Published by `realsense2_camera` driver. Namespace: `/camera/camera`.

Defined in `savo_realsense/constants.py` and `savo_realsense/ros/topic_contract.py`.

| Topic                                        | Message type                  | Hz   | Notes                              |
| -------------------------------------------- | ----------------------------- | ---- | ---------------------------------- |
| `/camera/camera/color/image_raw`             | `sensor_msgs/Image`           | 30   | RGB color stream (640×480)         |
| `/camera/camera/color/camera_info`           | `sensor_msgs/CameraInfo`      | 30   | Color camera intrinsics            |
| `/camera/camera/depth/image_rect_raw`        | `sensor_msgs/Image`           | 30   | Depth image, aligned to color      |
| `/camera/camera/depth/camera_info`           | `sensor_msgs/CameraInfo`      | 30   | Depth camera intrinsics            |
| `/camera/camera/depth/color/points`          | `sensor_msgs/PointCloud2`     | 10   | Colored point cloud (nav profile)  |

`align_depth.enable: true` is set in both D435 and VO profiles so depth and color are aligned in the same image plane.

`pointcloud.enable` is `true` for the D435 nav profile, `false` for the VO-only profile.

### Required topics for VO

rtabmap RGBD odometry requires:

```text
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/color/camera_info
```

### Required topics for depth front-min

`depth_front_min_node` requires:

```text
/camera/camera/depth/image_rect_raw
```

Output: `/depth/min_front_m` (`std_msgs/Float32`)

---

## Camera health and status topics

Published by `savo_realsense` monitoring nodes.

| Topic               | Message type                         | Publisher                   | Notes                              |
| ------------------- | ------------------------------------ | --------------------------- | ---------------------------------- |
| `/realsense/status` | `savo_msgs/CameraStatus` (TBD)       | `camera_health_node`        | Combined stream health status      |
| `/diagnostics`      | `diagnostic_msgs/DiagnosticArray`    | `camera_topic_monitor_node` | Per-topic rate and stale checks    |

---

## Base and odometry topics

Published by `savo_base` and `savo_localization` on `savo-core`.

| Topic                  | Message type              | Publisher           | Notes                               |
| ---------------------- | ------------------------- | ------------------- | ----------------------------------- |
| `/odom`                | `nav_msgs/Odometry`       | `savo_base`         | Wheel encoder odometry              |
| `/imu/data`            | `sensor_msgs/Imu`         | `savo_base`         | IMU data from onboard IMU           |
| `/odometry/filtered`   | `nav_msgs/Odometry`       | `savo_localization` | EKF-fused pose (wheel + IMU + VO)   |

---

## Motion control topics

| Topic           | Message type            | Notes                                               |
| --------------- | ----------------------- | --------------------------------------------------- |
| `/cmd_vel`      | `geometry_msgs/Twist`   | Raw velocity commands from Nav2, teleop, or intent  |
| `/cmd_vel_safe` | `geometry_msgs/Twist`   | Safety-gated commands passed to savo_base           |

Only `/cmd_vel_safe` reaches the motor board. The safety gate in `savo_perception` blocks `/cmd_vel` when obstacles are detected.

---

## Scan and map topics

| Topic       | Message type             | Publisher             | Notes                        |
| ----------- | ------------------------ | --------------------- | ---------------------------- |
| `/scan`     | `sensor_msgs/LaserScan`  | `savo_base` (LiDAR)   | 2D LiDAR for SLAM and Nav2   |
| `/map`      | `nav_msgs/OccupancyGrid` | `slam_toolbox`        | Built during mapping mode    |

---

## TF frame tree

```text
map
└── odom                    (EKF: savo_localization)
    └── base_link           (savo_base encoder odometry, corrected by EKF)
        ├── camera_link     (savo_description URDF static transform)
        │   ├── camera_color_optical_frame
        │   └── camera_depth_optical_frame
        ├── laser_frame     (savo_description URDF static transform)
        └── imu_link        (savo_description URDF static transform)
```

`odom → base_link` is published by the EKF node, not by `savo_base` directly. `savo_base` publishes `/odom` (the nav_msgs/Odometry message), which the EKF uses to produce the TF.

---

## QoS conventions

| Profile        | Reliability | Durability       | Used for                             |
| -------------- | ----------- | ---------------- | ------------------------------------ |
| Reliable       | Reliable    | Volatile         | Odometry, control commands           |
| Sensor data    | Best effort | Volatile         | Camera images, point clouds          |
| Status         | Best effort | Transient local  | Health status, diagnostics           |

RealSense driver publishes images with `sensor_data` QoS. Subscribing nodes should match this QoS profile.

---

## Related documents

- [`savo_edge_architecture.md`](savo_edge_architecture.md)
- [`localization_architecture.md`](localization_architecture.md)
- [`packages/savo_vo.md`](../packages/savo_vo.md)
- [`packages/savo_localization.md`](../packages/savo_localization.md)
