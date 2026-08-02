# Perception test plan

Prerequisites are safe-idle core bringup, verified sensor mounting/frames, a
static robot, and controlled obstacle fixtures. Required hardware is LiDAR,
VL53/ultrasonic sensors, head camera, AprilTags, and optionally D435.

Exact commands: run the package health tools, then
`python3 tools/diag/sensors/apriltag_test.py --output log/diag/apriltag.json`.
Keep D435 voxel disabled; validate filtered obstacle clouds separately.

Expected result: real, fresh, finite ranges and tag observations with correct
frames; safety state becomes conservative when a required input is stale.
Failure result is missing hardware, invalid timestamps, self/floor points, false
clearing, or a diagnostic timeout. Abort on unsafe slowdown/STOP behavior,
overheating, or unexpected actuation. Cleanup stops sensor nodes only and keeps
control in STOP. Record fixture distances, topic rates, frames, JSON, and bags
that contain no unnecessary private imagery.
