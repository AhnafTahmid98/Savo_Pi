# Full Robot SAVO staged test plan

No stage may be skipped. Record operator, UTC time, software revision, profile,
hardware serials, commands, logs, PASS/FAIL/BLOCKED, and abort reason. A failure
or abort blocks every later stage until reviewed.

| # | Stage | Prerequisites and hardware | Exact command or action | Expected result | Failure / abort condition | Cleanup and record |
| ---: | --- | --- | --- | --- | --- | --- |
| 1 | Static inspection | Repository checkout; no robot power | `deploy/common/validate_pre_real_test_readiness.sh` | Software report is PASS or physical-only BLOCKED | Any FAIL | Save JSON and human report |
| 2 | Power-off wiring inspection | Robot unpowered; wiring diagram; meter | Follow `docs/hardware/measurement_checklist.md` | Polarity, fusing, strain relief match drawings | Unknown polarity, short, loose wire | Keep power off; photograph and record |
| 3 | Core safe-idle boot | Core Pi only; E-stop accessible | `SAVO_ROBOT_MODE=safe_idle deploy/core/run_core.sh` | Control remains STOP; no motor motion | Any wheel motion, smoke, overcurrent, STOP missing | E-stop and remove drive power; save logs |
| 4 | Edge safe-idle boot | Edge Pi, display, camera, audio | `SAVO_ROBOT_MODE=safe_idle SAVO_START_UI=false deploy/edge/run_edge.sh` | Edge starts with optional devices honest/stale | Unexpected actuation or restart loop | Stop with Ctrl+C; save logs |
| 5 | Core-edge DDS discovery | Direct Ethernet; matching domain/RMW | `ros2 node list` on both Pis | Required peer nodes visible | Flapping discovery or wrong domain | Stop bringups; capture network state |
| 6 | Observer discovery | Trusted observer computer | `deploy/observer/run_observer.sh` | Read-only graph/dashboard connects | Observer publishes control topics | Stop observer; retain graph report |
| 7 | TF validation | Both Pis; sensors stationary | `python3 tools/diag/infra/tf_tree_check.py` | Required frames are connected and fresh | Missing looped or jumping transform | Stop test; save JSON |
| 8 | Sensor validation | LiDAR, IMU, encoders, range sensors, D435 | Run nonmoving sensor diagnostics individually | Real samples, timestamps, frames valid | Missing/stale/implausible data | Stop sensor nodes; save results |
| 9 | Power validation | UPS/base telemetry; meter | `python3 tools/diag/power/current_draw_logger.py` | All required rails and telemetry present | Undervoltage, excess current, overheating | Remove drive power; save trace |
| 10 | Audio validation | ReSpeaker and speaker; quiet area | Follow `docs/testing/speech_test_plan.md` | Capture/playback works with no feedback loop | Sustained feedback, clipping, missing gate | Mute speaker; save audio metadata only |
| 11 | UI validation | Edge display/touch | `python3 tools/diag/ui/screen_ui_test.py` | Live/stale states match ROS state | UI implies readiness when data is stale | Stop UI; save screenshots/logs |
| 12 | Wheels-raised motor validation | Geometry locked; robot secured on stands; two operators | Follow `docs/testing/base_test_plan.md`; run `motor_direction_test.py --allow-motion --wheels-raised` one direction at a time | Correct wheel direction at bounded command | Robot shifts, wrong wheel, STOP not immediate | E-stop, remove motor power, record video |
| 13 | Safety-stop validation | Stage 12 passed; obstacle fixtures | Run safety plan with drive raised | Every stop path forces zero output | Any nonzero gated velocity after stop | E-stop and remove power; save traces |
| 14 | Short guarded floor movement | Clear taped zone; spotter; E-stop | Approved bounded control test only | Slow commanded motion matches direction | Person enters zone, drift, localization jump | E-stop; return robot manually |
| 15 | Manual mapping | Stages 7–14 passed | `ros2 launch savo_bringup manual_mapping.launch.py` | Map session records and saves without bypass | Localization/safety/release error | Cancel mapping; preserve session logs |
| 16 | Map save and verification | Completed manual session | Use typed map save and verification interfaces | Artifacts and hashes verify | Missing artifact/hash/quality report | Do not promote; archive failure |
| 17 | Autonomous mapping | Manual flow proven; guarded area | `ros2 launch savo_bringup autonomous_mapping.launch.py` | Authorized mission completes/cancels safely | Safety, localization, coverage, timeout fault | STOP/cancel; retain recovery journal |
| 18 | AprilTag registration and review | Verified map; physical tags | Use mapped-location registration and operator review | Candidate correlates to tag/map/session | Ambiguous tag or unreviewed candidate | Reject candidate; save evidence |
| 19 | AM-8 approval and release | All AM-8 inputs valid; authorized operator | Use documented correlated review CLI | Real release ID and atomic active context | Any gate missing or mismatched correlation | Do not retry blindly; preserve journal |
| 20 | Saved-map LiDAR-only navigation | Approved release; AMCL ready | Launch production navigation with voxel disabled | Named approved goal admitted and reached | Wrong map/release, safety/readiness loss | Cancel/STOP; save Nav2 and supervisor logs |
| 21 | Cancellation and recovery tests | Stage 20 passed | Cancel typed navigation at defined points | Zero motion and recoverable STOP state | Continued motion or stale active command | E-stop; save correlation IDs |
| 22 | D435 filtering validation | LiDAR navigation proven | Enable obstacle-cloud producer, not voxel layer | Filtered points/frames/bandwidth meet limits | Floor/self points or stale cloud | Disable D435 producer; retain bag/log |
| 23 | Optional voxel profile validation | Stage 22 signed off; geometry locked | Explicit validated voxel profile only | Obstacles affect costmap without false blocks | Any unsafe clearing/marking or overload | Disable voxel profile and revert to LiDAR |

The plan never grants approval to lock geometry or AM-8 automatically.

## Hardware-free software gate

Before copying the workspace to either Pi, run the complete affected-package
build and test gate on the ROS 2 Jazzy development PC:

```bash
cd ~/Savo_Pi
deploy/common/run_pre_real_test_regression.sh --clean-affected
```

This command performs source validation, dependency checking, affected-package
builds/tests, and launch-argument generation. It never launches robot nodes. A
`BLOCKED` readiness result caused only by provisional geometry or other physical
prerequisites is expected; any repository `FAIL` must be corrected first.
