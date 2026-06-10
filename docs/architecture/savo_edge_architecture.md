# savo-edge Architecture

`savo-edge` is the helper and heavier-processing Raspberry Pi in the two-Pi robot split. It does not own motion authority but provides the sensor data, visual odometry, speech, UI, and AI helper services that support `savo-core`.

See [`two_pi_architecture.md`](two_pi_architecture.md) for the full two-Pi split overview.

## Role summary

| Responsibility              | Detail                                                                          |
| --------------------------- | ------------------------------------------------------------------------------- |
| RealSense/depth camera      | Runs `realsense2_camera` driver + `savo_realsense` monitoring nodes             |
| Visual odometry (VO)        | Runs `rtabmap_ros` RGBD odometry + `savo_vo` republisher and health nodes       |
| Speech and audio            | Runs `savo_speech` for TTS/STT bridging and audio I/O                           |
| Robot UI display            | Runs `savo_ui` for the robot-side display panel                                 |
| AI/intent helper            | Communicates with `Robot_Savo_Server` Docker services over HTTP                 |
| Diagnostics relay           | Publishes camera and VO health to `/realsense/status`, `/vo/status`, `/diagnostics` |

## Visual odometry role on savo-edge

`savo-edge` owns the full VO pipeline. This is a deliberate placement choice because VO is computationally heavy, requires the RealSense camera directly, and is non-critical to the safety stop path.

The VO pipeline on `savo-edge`:

```text
RealSense D435 (USB 3)
        ↓
realsense2_camera driver
        ↓
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/color/camera_info
        ↓
rtabmap_ros (rgbd_odometry node)
        ↓
/vo/odom_raw   (internal, rtabmap frame)
        ↓
savo_vo vo_republisher_node
        ↓
/vo/odom   (nav_msgs/Odometry, odom → camera_link)
        ↓
  [published over ROS 2 network to savo-core]
        ↓
savo_localization EKF on savo-core
```

`savo-edge` is responsible for:

- Running and monitoring the RealSense camera streams
- Running `rtabmap_ros` RGBD odometry with the VO profile config
- Republishing the VO output as `/vo/odom` with the correct frames and QoS
- Monitoring VO health and publishing `/vo/status`
- Logging VO diagnostics to `/diagnostics`

`savo-core` is responsible for:

- Consuming `/vo/odom` as one input to the EKF
- Fusing VO with wheel odometry and IMU
- Owning the final pose estimate at `/odometry/filtered`

## ROS 2 packages on savo-edge

| Package            | Role on savo-edge                                            |
| ------------------ | ------------------------------------------------------------ |
| `savo_msgs`        | Shared custom message definitions                            |
| `savo_description` | URDF, TF frames, RViz views shared with savo-core            |
| `savo_perception`  | RealSense depth nodes, depth front-min helper                |
| `savo_realsense`   | Camera driver bringup, monitoring, health, and diagnostics   |
| `savo_vo`          | VO republisher, health, and diagnostics nodes                |
| `savo_speech`      | TTS/STT bridge, audio I/O                                    |
| `savo_ui`          | Robot display UI                                             |
| `savo_intent`      | Intent helper node communicating with AI server              |
| `savo_dashboard`   | Optional diagnostics dashboard                               |
| `savo_bringup`     | Edge-role launch profiles                                    |

## Launch profiles

| Launch file                                    | What it brings up                                               |
| ---------------------------------------------- | --------------------------------------------------------------- |
| `savo_realsense/realsense_vo.launch.py`        | Camera driver + topic monitor + health node (VO profile)        |
| `savo_realsense/realsense_bringup.launch.py`   | Camera driver + monitor + health + depth_front_min (full D435)  |
| `savo_vo/vo_bringup.launch.py`                 | rtabmap RGBD odometry + VO republisher + health + diagnostics   |
| `savo_vo/rgbd_odometry.launch.py`              | rtabmap RGBD odometry only                                      |
| `savo_vo/vo_diagnostics.launch.py`             | VO diagnostics node only                                        |

## Motion authority boundary

`savo-edge` does not send commands to the motor board.

Any motion intent from edge-side systems — speech commands, AI actions, UI buttons — must pass through `savo-core` control and safety layers before reaching the base driver.

```text
savo-edge (intent / speech / AI)
        ↓ ROS 2 network
savo_control on savo-core
        ↓
/cmd_vel → safety gate → /cmd_vel_safe → savo_base → motors
```

The robot must be able to stop safely even if `savo-edge`, the RealSense camera, or `Robot_Savo_Server` fails.

## Failure isolation

| Failure                          | Impact on savo-core                                            |
| -------------------------------- | -------------------------------------------------------------- |
| RealSense disconnects            | `/vo/odom` stops; EKF falls back to wheel + IMU odometry       |
| savo-edge crashes                | VO, speech, UI, AI go down; core continues moving safely       |
| Robot_Savo_Server unreachable    | Intent/AI unavailable; robot stays in last safe state          |
| VO drift or bad pose             | Core should detect stale odom and reduce reliance on VO        |

VO is a supplemental odometry input, not a safety-critical input. Loss of VO should degrade localization quality, not stop the robot.

## Related documents

- [`two_pi_architecture.md`](two_pi_architecture.md)
- [`localization_architecture.md`](localization_architecture.md)
- [`ros2_topic_contracts.md`](ros2_topic_contracts.md)
- [`packages/savo_vo.md`](../packages/savo_vo.md)
- [`setup/realsense_setup.md`](../setup/realsense_setup.md)
