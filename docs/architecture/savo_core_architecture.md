# SAVO core architecture

Core is the sole host for drivetrain, safety, LiDAR/core sensors, control,
localization, mapping, Nav2, locations, supervisor, head, and core/base power.
Bringup starts fail-closed in `safe_idle`, `lidar_only`, and control `STOP`.

Navigation requires verified active release artifacts, locked geometry,
supervisor map-context synchronization, readiness, goal admission, and then Nav2.
Mapping release requires save, verification, quality evaluation, location review,
correlated operator approval, and atomic AM-8 release. Edge or SavoMind cannot
bypass these authorities.
