# Perception Architecture

Perception is split between mandatory Core near-field safety and optional Edge depth processing.

## Core path

```text
TCA9548A ch2 -> left VL53L1X --+
TCA9548A ch3 -> right VL53L1X -+-> range health -> safety stop/slowdown
GPIO 27/22 -> front ultrasonic -+                         |
/cmd_vel --------------------------------------------------+-> /cmd_vel_safe
```

The C++ Core nodes own I2C bus 1, multiplexer `0x70`, VL53L1X address `0x29` behind each selected channel, ultrasonic trigger GPIO 27 and echo GPIO 22, range health, safety fusion, and the command gate. Required Core sensors are left/right ToF; front depth is not a substitute.

## Edge path

RealSense D435 produces aligned RGB-D and point-cloud inputs. Edge may filter `/camera/camera/depth/color/points` into `/savo_perception/obstacles/points`, publish `/depth/min_front_m`, and expose cloud health/status/heartbeat. This path is disabled by default and the `lidar_d435_voxel` bringup profile is physically gated by `d435_voxel_validated`.

Fixed camera and sensor TF comes from `savo_description`; RealSense TF publication is disabled. Sensor mounts are source-configured but provisional, so spatial filtering/costmap claims depend on locked measurements.

## Degraded behavior and evidence

Core range staleness/failure is fail-safe. An optional Edge cloud failure must degrade or block only profiles that require it and must not suppress Core stops. High-bandwidth data should stay Edge-local unless explicitly required across Ethernet.

Unit/source contracts cover topics, gating, heartbeat, and launch requirements. Physical validation remains for field of view, cross-talk, sunlight/material response, ultrasonic echo behavior, mount occlusion, clock alignment, D435 USB bandwidth/depth quality, cloud frame alignment, threshold tuning, and stopping distance.
