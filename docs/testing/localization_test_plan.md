# Localization test plan

Prerequisites are measured geometry, stationary core safe-idle, calibrated IMU,
verified encoder direction, and a connected TF tree. Required hardware is the
BNO055, four encoders, raised-wheel fixture, and later a measured floor course.

Exact commands: `ros2 launch savo_localization diagnostics.launch.py` and
`python3 tools/diag/infra/tf_tree_check.py`; use `savo_observer` for visualization.
Run odometry calibration only with `--allow-motion` after the base plan passes.

Expected result: C++ health reports fresh IMU/wheel/EKF data, TF has one connected
authority, covariance is finite, and stopped drift stays within the recorded
tolerance. Missing sensors, jumps, duplicate TF, implausible covariance, or scale
error is FAIL/BLOCKED. Abort any moving portion on safety loss or localization
jump. Cleanup returns STOP and terminates launch with Ctrl+C. Record bags, plots,
geometry revision, course measurements, timestamps, and results.
