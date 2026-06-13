# savo_lidar

`savo_lidar` owns the RPLIDAR A1 hardware layer for Robot Savo.

This package brings up the LiDAR, publishes scan data, filters scan ranges, monitors scan freshness, reports LiDAR health, and provides diagnostic tools for PC and Pi testing.

It does not own SLAM, Nav2, robot movement, or final safety-stop decisions. Mapping and navigation packages consume its scan output through ROS topics.

## Responsibility

`savo_lidar` is responsible for:

- RPLIDAR A1 bringup
- USB serial port validation
- `/scan` publishing
- optional `/scan/filtered` publishing
- scan quality diagnostics
- scan watchdog state
- LiDAR health summary
- dryrun scan generation for PC tests

`savo_lidar` is not responsible for:

- map creation
- Nav2 planning
- costmap ownership
- motor control
- final `/cmd_vel_safe` safety gating

## Hardware ownership

| Hardware | Owner |
| --- | --- |
| RPLIDAR A1 | `savo_lidar` |
| Freenove motor board | `savo_base` |
| Intel RealSense | `savo_realsense` |
| ToF / ultrasonic safety sensors | `savo_perception` |
| IMU / wheel odometry / EKF | `savo_localization` |

## ROS topic contract

Published by this package:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Main LiDAR scan output |
| `/scan/filtered` | `sensor_msgs/msg/LaserScan` | Optional filtered scan output |
| `/savo_lidar/state` | `std_msgs/msg/String` | Driver state as compact JSON |
| `/savo_lidar/scan_quality` | `std_msgs/msg/String` | Scan quality summary as compact JSON |
| `/savo_lidar/watchdog_state` | `std_msgs/msg/String` | Scan freshness state as compact JSON |
| `/savo_lidar/health` | `std_msgs/msg/String` | Combined LiDAR health status |
| `/savo_lidar/state_summary` | `std_msgs/msg/String` | Dashboard-friendly summary |

Consumed by downstream packages:

| Package | Uses |
| --- | --- |
| `savo_mapping` | `/scan` for SLAM and map creation |
| `savo_nav` | `/scan` for Nav2 costmaps and obstacle awareness |
| `savo_perception` | Optional LiDAR diagnostics or sector awareness |
| `savo_ui` | LiDAR health/status display |

## Frames

The scan frame is:

```text
laser
```

`savo_lidar` only publishes scan messages with:

```text
frame_id: laser
```

The physical transform must be defined in `savo_description`:

```text
base_link -> laser
```

Do not duplicate LiDAR position or orientation inside `savo_lidar`. The robot description package is the source of truth for fixed sensor geometry.

## Profiles

Profile files are stored in:

```text
config/profiles/
```

Available profiles:

| Profile | Purpose |
| --- | --- |
| `dryrun_sim.yaml` | Safe PC test with synthetic scan data |
| `bench_test.yaml` | Real RPLIDAR hardware bench test |
| `real_rplidar_a1.yaml` | Normal robot runtime profile |
| `mapping_rplidar_a1.yaml` | Stricter scan checks for mapping sessions |

## PC dryrun test

Build first:

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select savo_lidar --symlink-install
source install/setup.bash
```

Run dryrun LiDAR stack:

```bash
ros2 launch savo_lidar lidar_dryrun_test.launch.py
```

Check nodes:

```bash
ros2 node list
```

Expected nodes:

```text
/lidar_driver_node
/lidar_filter_node
/lidar_watchdog_node
/lidar_health_node
/lidar_state_publisher_node
```

Check topics:

```bash
ros2 topic list
```

Expected important topics:

```text
/scan
/scan/filtered
/savo_lidar/state
/savo_lidar/scan_quality
/savo_lidar/watchdog_state
/savo_lidar/health
/savo_lidar/state_summary
```

Print compact scan summaries:

```bash
ros2 run savo_lidar scan_echo_cli.py --samples 3
```

Check scan quality:

```bash
ros2 run savo_lidar scan_quality_cli.py --samples 3
```

## Real RPLIDAR test on Pi

Before starting the real driver, check the serial port:

```bash
ls /dev/ttyUSB*
groups
```

The user should be in the `dialout` group.

Run the port diagnostic:

```bash
ros2 run savo_lidar find_lidar_port_cli.py --serial-port /dev/ttyUSB0
```

Run hardware-only bringup:

```bash
ros2 launch savo_lidar lidar_hw_only.launch.py profile:=bench_test.yaml
```

Then check:

```bash
ros2 topic echo /scan --once
```

For full LiDAR bringup:

```bash
ros2 launch savo_lidar lidar_bringup.launch.py profile:=real_rplidar_a1.yaml
```

## Mapping-ready LiDAR bringup

Before starting `savo_mapping`, run:

```bash
ros2 launch savo_lidar lidar_mapping_ready.launch.py
```

This uses stricter quality and stale-scan thresholds so bad scan data is detected before mapping begins.

## Development notes

Keep this package focused on LiDAR hardware ownership and scan health.

Do not add SLAM logic here.  
Do not add Nav2 costmap logic here.  
Do not add base movement logic here.  
Do not store physical sensor position here.

The clean data path is:

```text
savo_lidar
  publishes /scan
      ↓
savo_mapping
  builds maps
      ↓
savo_nav
  plans and navigates
      ↓
savo_control
  selects and shapes velocity commands
      ↓
savo_perception
  applies safety gate
      ↓
savo_base
  drives the motors
```

## Current status

This package is under staged development.

Initial target:

```text
PC dryrun build and scan test
```

Next target:

```text
Pi RPLIDAR A1 bench test
```

Final target:

```text
Mapping-ready LiDAR bringup for Robot Savo
```
