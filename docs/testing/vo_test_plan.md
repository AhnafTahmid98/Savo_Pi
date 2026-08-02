# Visual odometry test plan

Prerequisites are edge safe-idle, mounted/calibrated D435, correct optical TF,
time synchronization, and LiDAR/localization already validated. Required hardware
is the D435 and a measured static/motion scene.

Exact command: launch the existing `savo_vo` bench profile with control STOP and
observe its odometry/diagnostics in `savo_observer`. Do not enable Nav2 voxel mode.

Expected result: fresh finite poses, monotonic stamps, correct frames/covariance,
bounded drift, and honest stale state on camera loss. Missing camera, timestamp
regression, frame mismatch, jump, excessive CPU/bandwidth, or silent stale output
is FAIL/BLOCKED. Abort on thermal limits or any unintended motion. Cleanup stops
VO and camera nodes, leaves LiDAR-only defaults, and records calibration, rates,
drift, CPU, logs, and result.
