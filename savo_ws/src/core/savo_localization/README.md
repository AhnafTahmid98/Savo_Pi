# Robot SAVO localization

The production core localization path is C++ IMU publishing, C++ four-wheel
odometry, `robot_localization` EKF, and the C++ `localization_health_node`.
`diagnostics.launch.py` starts the health node only; live visualization belongs
to the read-only `savo_observer` dashboard and RViz profile.

The old empty Python dashboard, EKF-state publisher, and wheel odometry fallback
were removed. There is deliberately no wheel odometry fallback executable: a
missing hardware encoder node must block production startup rather than silently
changing localization authority.

Safe inspection:

```bash
ros2 launch savo_localization diagnostics.launch.py --show-args
ros2 launch savo_observer observer.launch.py --show-args
```

These commands only display launch arguments when used with `--show-args`.
