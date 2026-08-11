# Sensor Mounting

Coordinates use +X forward, +Y left, +Z up. Ground XYZ values are owner
measurements; `base_link` XYZ values are derived by subtracting the `0.0325 m`
axle-plane frame height.

| Device/frame | Measured ground XYZ (m) | Derived `base_link` XYZ (m) | RPY / state |
| --- | --- | --- | --- |
| RPLIDAR / `laser_frame` | `[0,0,0.330]` | `[0,0,0.2975]` | roll/pitch provisionally zero; yaw requires `/scan` validation |
| BNO055 / `imu_link` | `[0,-0.0465,0.015]` | `[0,-0.0465,-0.0175]` | +Z up known; +X/+Y orientation unresolved, zero RPY is a placeholder |
| D435 / `camera_link` | `[0.130,0,0.225]` | `[0.130,0,0.1925]` | mount RPY measured `[0,0,0]` |
| Left ToF | `[0,+0.106,0.025]` | `[0,+0.106,-0.0075]` | yaw `+pi/2`, sensing +Y |
| Right ToF | `[0,-0.106,0.025]` | `[0,-0.106,-0.0075]` | yaw `-pi/2`, sensing -Y |
| Front ultrasonic | `[0.137,0,0.056]` | `[0.137,0,0.0235]` | yaw zero, sensing +X |
| Pan axis / `pantilt_mount_link` | `[0.115,0,0.244]` | `[0.115,0,0.2115]` | pan axis +Z; servo sign unverified |

The neutral head chain continues from the description-owned mount with
`mount->pan [0,0,0]`, `pan->tilt [0,0,0.046]`, and
`tilt->pi_camera [0.025,0,-0.010] m`. It reconstructs the measured tilt axis
`[0.115,0,0.290]` and lens center `[0.140,0,0.280] m` above ground.

`robot_state_publisher` remains the sole D435 TF authority and RealSense keeps
`publish_tf=false`. The zero translations between `camera_link`, color, and
depth frames are provisional modeling placeholders, not calibrated D435 stream
extrinsics. Display and ReSpeaker positions remain legacy provisional values.
