# Robot SAVO localization

The production core localization path is C++ IMU publishing, C++ four-wheel
odometry, `robot_localization` EKF, and the C++ `localization_health_node`.
`diagnostics.launch.py` starts the health node only; live visualization belongs
to the read-only `savo_observer` dashboard and RViz profile.

The old empty Python dashboard, EKF-state publisher, and wheel odometry fallback
were removed. There is deliberately no wheel odometry fallback executable: a
missing hardware encoder node must block production startup rather than silently
changing localization authority.

The C++ wheel odometry node also publishes `/joint_states` for the four
continuous wheel joints. Position and velocity are measured from the same
signed encoder sample used by wheel odometry and are expressed in radians and
radians per second. This passive state output lets `robot_state_publisher` and
RViz update the wheel links; it does not command the motors and does not publish
wheel-link TF itself.

Safe inspection:

```bash
ros2 launch savo_localization diagnostics.launch.py --show-args
ros2 launch savo_observer observer.launch.py --show-args
```

These commands only display launch arguments when used with `--show-args`.

## BNO055 calibration persistence

The production IMU policy keeps `reset_on_start: true` and restores the actual
22-byte BNO055 page-0 offset/radius profile before entering the configured
operational mode. The default persistent path is:

```text
/var/lib/robot_savo/localization/bno055_calibration.yaml
```

The profile is versioned YAML containing the I2C bus/address, operational mode,
capture timestamp, signed accelerometer/magnetometer/gyroscope offsets, and
unsigned accelerometer/magnetometer radii. No production calibration values are
checked into the repository.

On the Core Pi, `deploy/core/prepare_runtime_storage.sh` explicitly creates
`/var/lib/robot_savo/localization` with mode `0750` and ownership matching the
runtime user/group. The calibration save requests mode `0640` for the YAML
(subject to any stricter process umask). Production startup only reads this
path; saving remains an explicit operator action.

A missing profile is a normal first-run state: the IMU continues publishing and
calibrating live. A malformed, incompatible, or unverified profile is reported
as a restore failure and is an error when
`calibration_require_verified_restore` is true. Successful register readback
does not fake `CALIB_STAT` or override live sensor health.

Restoration can be deliberately disabled for controlled bench work:

```bash
ros2 launch savo_localization localization_bench_imu.launch.py \
  calibration_restore_enabled:=false
```

Calibration is never saved automatically. After the live status reaches exactly
`system=3`, `gyro=3`, `accel=3`, and `mag=3`, an operator can request one atomic
capture:

```bash
ros2 service call /savo_localization/save_imu_calibration std_srvs/srv/Trigger '{}'
```

The service reads the complete register block in CONFIG mode, restores the
original operational mode, validates every documented range, writes and fsyncs
a temporary YAML file, atomically renames it, and returns the captured offsets
and radii for audit. A failed request does not intentionally replace an existing
valid profile.

Restore state is exposed in `/savo_localization/imu_state` and `/diagnostics`,
including profile presence/load, restore attempt, exact readback verification,
operational-status verification, and failure detail. The established warning
`IMU usable, calibration not fully ready` remains driven by live BNO055 status.
