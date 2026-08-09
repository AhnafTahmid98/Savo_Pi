# Savo Core Architecture

Core is the safety and autonomy computer. Its production set contains `savo_msgs`, `savo_description`, `savo_bringup`, `savo_perception`, `savo_power`, `savo_supervisor`, `savo_base`, `savo_control`, `savo_lidar`, `savo_localization`, `savo_locations`, `savo_mapping`, `savo_nav`, and `savo_head`.

## Responsibility graph

```text
sensors -> localization / perception / power
                    |          |
                    v          v
request -> supervisor permission -> operation owner
                                   |
                    navigation/mapping/manual lane
                                   v
                              savo_control
                                   v
                          perception safety gate
                                   v
                              savo_base
```

`savo_base` alone owns the motor-board outputs. `savo_control` owns mode, lane selection, shaping, rotation/approach, and recovery coordination. `savo_perception` owns near-field stop/slowdown and the `/cmd_vel` to `/cmd_vel_safe` gate. `savo_supervisor` owns arming, fault latch, system permission, map-context authorization, and persistent system state—but never operation execution.

## Hardware and TF ownership

Core owns the RPLIDAR serial device, BNO055, four encoder pairs, ToF multiplexer/sensors, ultrasonic GPIO, base-battery ADC, Core UPS monitor, Freenove motor PCA9685 channels, pan/tilt PCA9685 channels, and Pi Camera 2 NoIR pipeline. The base and head currently open the same PCA9685 (`bus 1`, `0x40`) and initialize its shared frequency; channel ranges do not overlap, but concurrent initialization/reset behavior requires physical integration validation.

Core `robot_state_publisher` owns fixed URDF transforms. The EKF owns `odom -> base_footprint`. SLAM or AMCL, never both, owns `map -> odom`. Head dynamic TF is disabled until its transforms are calibrated and explicitly enabled.

## Startup and degraded behavior

Core bringup validates role/mode/profile, geometry policy, required component freshness, safety clear state, localization, map context, navigation, and supervisor authority as applicable. A required failure or startup timeout blocks readiness. Sensor staleness and command watchdogs fail closed. Edge loss can remove VO or high-level interaction without granting alternate motion authority.

Persistent Core artifacts live under `/var/lib/robot_savo`; logs live under `/var/log/robot_savo`. The production wrapper prepares map sessions/releases, location state, supervisor state, ROS home, and log directories.

Source validators cover the launch/authority contracts. Remaining gates include locked geometry, the wheel-geometry/kinematics discrepancy, shared-PCA9685 integration, sensor polarity/rates, motor watchdog, power calibration, and guarded real-robot regression.
