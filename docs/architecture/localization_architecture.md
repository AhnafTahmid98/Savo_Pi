# Localization Architecture

Localization architecture, frame ownership, odometry sources, and state-estimation boundaries.

## Overview

`savo-core` owns all localization. It fuses multiple odometry sources into a single filtered pose estimate using robot_localization EKF, then publishes the authoritative robot position for Nav2 and mapping.

`savo-edge` contributes visual odometry (`/vo/odom`) as one input to the EKF over the shared ROS 2 network.

## Odometry sources

| Source             | Topic              | Type                    | Owner        | Notes                                   |
| ------------------ | ------------------ | ----------------------- | ------------ | --------------------------------------- |
| Wheel odometry     | `/odom`            | `nav_msgs/Odometry`     | `savo-core`  | From four mecanum encoders in savo_base |
| IMU                | `/imu/data`        | `sensor_msgs/Imu`       | `savo-core`  | From IMU hardware on savo_base          |
| Visual odometry    | `/vo/odom`         | `nav_msgs/Odometry`     | `savo-edge`  | From rtabmap RGBD + savo_vo republisher |

LiDAR scan data from `/scan` feeds SLAM (slam_toolbox) during mapping, not the EKF directly.

## /vo/odom fusion plan

`/vo/odom` is published by `savo_vo` on `savo-edge` and consumed by the EKF on `savo-core` over the ROS 2 network.

### What the EKF expects from /vo/odom

| Field                  | Expected value                                    |
| ---------------------- | ------------------------------------------------- |
| `header.frame_id`      | `odom`                                            |
| `child_frame_id`       | `camera_link`                                     |
| Message type           | `nav_msgs/Odometry`                               |
| Rate                   | 10–15 Hz (governed by rtabmap RGBD odometry)      |
| Covariance             | Non-zero; reflects visual confidence              |

### Fusion strategy

The EKF fuses position and velocity estimates from all sources. VO supplements wheel + IMU by providing drift correction in the XY plane and some yaw correction, particularly useful in featureless corridors where wheel slip accumulates.

Recommended fusion configuration:

| Source       | EKF role                                | Primary correction axes |
| ------------ | --------------------------------------- | ----------------------- |
| Wheel odom   | Primary position source, high weight    | X, Y, Yaw              |
| IMU          | Orientation and angular rate, high rate | Roll, Pitch, Yaw rate   |
| VO           | Drift correction supplement, lower rate | X, Y, Yaw              |

VO is weighted lower than wheel odometry by default. It should help, not dominate the pose estimate.

### What happens when /vo/odom is absent

The EKF should be configured with a timeout so that if `/vo/odom` stops arriving (RealSense disconnect, savo-edge crash), the EKF continues fusing wheel + IMU without blocking or crashing.

VO loss causes:

- Slightly higher drift in long corridors
- No change to safety stop behavior
- Nav2 continues using the filtered estimate

The robot must navigate safely without VO. VO is a quality improvement, not a hard dependency.

## EKF output

| Topic                  | Type                  | Description                                     |
| ---------------------- | --------------------- | ----------------------------------------------- |
| `/odometry/filtered`   | `nav_msgs/Odometry`   | Fused pose estimate for Nav2 and mapping        |
| `/tf`                  | TF2 broadcast         | `odom → base_link` transform                   |

Nav2 and slam_toolbox subscribe to `/odometry/filtered` and the `odom → base_link` TF.

## Frame ownership

| Frame                     | Owner                  | Notes                                           |
| ------------------------- | ---------------------- | ----------------------------------------------- |
| `map`                     | slam_toolbox / AMCL    | Established during mapping or localization      |
| `odom`                    | robot_localization EKF | Published by savo_localization on savo-core     |
| `base_link`               | savo_base              | Robot center frame; origin of physical model    |
| `camera_link`             | savo_description URDF  | Fixed transform from base_link                  |
| `camera_color_optical_frame` | savo_description     | Optical frame for color image                   |
| `camera_depth_optical_frame` | savo_description     | Optical frame for depth image                   |

The `odom → base_link` transform is published by the EKF. No other node should publish this transform.

The `base_link → camera_link` transform is published by `robot_state_publisher` using the URDF from `savo_description`.

## SLAM vs localization-only mode

| Mode              | Map source                       | Pose method                            |
| ----------------- | -------------------------------- | -------------------------------------- |
| Mapping mode      | slam_toolbox builds map live     | SLAM internal pose + EKF               |
| Localization mode | Pre-built map loaded from disk   | AMCL or slam_toolbox localization-only |

During mapping, the EKF still runs. SLAM uses the filtered odometry as its motion model.

## Related documents

- [`two_pi_architecture.md`](two_pi_architecture.md)
- [`savo_edge_architecture.md`](savo_edge_architecture.md)
- [`ros2_topic_contracts.md`](ros2_topic_contracts.md)
- [`packages/savo_vo.md`](../packages/savo_vo.md)
- [`packages/savo_localization.md`](../packages/savo_localization.md)
- [`testing/vo_test_plan.md`](../testing/vo_test_plan.md)
