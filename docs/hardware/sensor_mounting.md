# Sensor Mounting

Coordinates use ROS convention: x forward, y left, z up, metres/radians, relative to `base_link`. Values are current source configuration and are **provisional**, not measured installation records.

| Device/frame | Configured xyz (m) | Configured RPY | Mount/validation concern |
| --- | --- | --- | --- |
| RPLIDAR / `laser_frame` | `[0, 0, 0.205]` | `[0,0,0]` | 360-degree clearance, level, vibration |
| BNO055 / `imu_link` | `[0, 0, 0.075]` | `[0,0,0]` | Axis alignment, magnetic/motor interference |
| D435 / `camera_link` | `[0.135, 0, 0.155]` | `[0,0,0]` | USB strain, depth FOV, optical convention |
| Left ToF | `[0.125, 0.090, 0.070]` | `[0,0,0]` | Side/forward aim and cross-talk |
| Right ToF | `[0.125, -0.090, 0.070]` | `[0,0,0]` | Side/forward aim and cross-talk |
| Front ultrasonic | `[0.150, 0, 0.070]` | `[0,0,0]` | Beam cone, floor/bumper echoes |
| Display | `[0.115, 0, 0.145]` | `[0,0,0]` | Visibility, touch load, cable flex |
| ReSpeaker | `[0, 0, 0.180]` | `[0,0,0]` | Acoustic clearance and vibration |

The description also places a fixed `depth_obstacle_frame` at the D435 translation. Head mount offsets are currently zero, dynamic transforms are marked uncalibrated, and head TF publication is disabled. Pi camera extrinsics must be measured before enabling that chain.

Measure each origin from the defined `base_link` datum, photograph the datum and axes, record tool uncertainty, verify orientation with live data, update the geometry profile, regenerate the URDF/footprint, review the digest, and lock only after independent inspection. Revalidate after any bracket, deck, wheel, or sensor change.
