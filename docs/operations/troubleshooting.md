# Troubleshooting

Start with non-destructive observation, keep control in `STOP`, preserve logs,
and escalate rather than bypassing a gate. Motion symptoms require the physical
stop control and clear area before checks.

| Symptom | Likely area | Safe checks | Recovery | Escalate when |
| --- | --- | --- | --- | --- |
| Core unit will not start | Storage, environment, build | `systemctl status`; Core journal; writable state roots | Correct documented path/permission; restart once | Repeats, crashes, or startup mode is not STOP |
| Edge unit will not start | Runtime sockets, device, build | Edge journal; `/run` ownership; install prefix | Prepare approved sockets; restart once | Repeats or bridge identity differs |
| Duplicate node/component | Two launch/service owners | `ros2 node list`; `systemctl list-units 'savo*'` | Stop unintended owner while in STOP | Ownership is unclear or duplicate command/TF publisher |
| Role mismatch | Wrong host/env | `hostname -s`; service environment; journal | Use correct installed role | Hardware package ran on wrong host |
| Runtime directory failure | Ownership/provisioning | `ls -ld` relevant `/run` or state path | Maintainer reruns approved preparation | Ownership keeps changing |
| Core cannot see Edge | Ethernet/DDS | Network summary; ping peer; domain/RMW | Restore approved link/config | DDS remains absent or link flaps |
| Clock offset | Chrony/network | `chronyc tracking`; `sources -v` | Restore Core-preferred sync | Offset recurs or timestamps invalidated motion |
| Observer cannot connect | DDS/workstation | Observer connection script; domain/network | Correct observer environment | Would require exposing DDS publicly |
| Robot does not move | STOP/authority/mux/safety | Mode/source topics; supervisor; `/safety/stop`; safe output | Resolve gate; reauthorize normally | Direct output test seems necessary |
| Wrong wheel direction | Motor/encoder config or wiring | Stop/isolate; compare wheel identity/evidence | Maintainer wheels-raised validation | Any floor motion or config ambiguity |
| Immediate safety stop | Range health/obstacle/stale data | Safety/range health and physical field | Remove real obstacle; repair sensor | False clear/false stop persists |
| Command expires | Publisher/watchdog/network | Source/mux/shaper/base status | Restore supported publisher; remain STOP | Watchdog does not zero output |
| Control stuck in STOP | External stop, policy, fault | Mode reason; external stop; supervisor state | Clear root cause, then explicit release | Unknown transition or mode bypass proposed |
| Supervisor denies | Readiness/fault/map context | State summary/capabilities/readiness | Correct dependency; approved clear/arm | Fault latch or state mismatch unexplained |
| No LiDAR | USB/serial/driver | Journal, kernel log, `/scan`, device permissions | Reseat while powered down; restart owner | Repeated disconnect or wrong device identity |
| ToF unavailable | I2C mux/sensor | Range health; I2C owner logs | Inspect power/cable; component validation | Bus conflict or unsafe false clear |
| Ultrasonic unavailable | GPIO/level/wiring | Range health and owner logs | Inspect with power isolated | Echo voltage or pin allocation uncertain |
| IMU unavailable | I2C/driver | Localization health and IMU topic | Inspect/restart owner in STOP | Axis/calibration or bus fault |
| Encoder stale | GPIO/wheel/driver | Wheel odom/health; isolated connector | Wheels-raised component validation | Counts/polarity inconsistent |
| RealSense unavailable | USB3/config | Edge journal; status; kernel USB | Restore approved cable/port; restart Edge once | Serial/profile mismatch or repeated reset |
| VO unhealthy | RGB-D/time/quality | VO health, camera status, Chrony | Disable optional fusion/profile through release config | VO is required or scale/frame is wrong |
| Pose jumps or drifts | Encoder/IMU/VO/EKF | Odom/health, TF, timestamps | Stop navigation; recalibrate/revalidate | Jump occurred during motion |
| Missing `map -> odom` | SLAM/AMCL mode | TF echo; active mode; node list | Start correct single authority | Both or neither owners appear unexpectedly |
| Missing `odom -> base_footprint` | EKF/localization | Localization health; TF echo | Restore localization owner | Duplicate TF or recurring loss |
| Map not updating | SLAM/scan/TF | Map topic, scan, TF, mapping status | Stop motion; restore inputs | Loop closure/pose becomes unsafe |
| Map save fails | Storage/map saver | Storage preflight; session status; journal | Correct space/permission; preserve session | Partial/corrupt artifacts |
| Quality/release fails | Evidence/context/digest | Quality report and release journal | Keep unreleased; repeat approved workflow | Hash/context or rollback failure |
| Navigation goal rejected | Readiness/map/authority | Nav readiness/reason, map context, supervisor | Correct gate; resubmit only after ready | Internal action bypass is suggested |
| No path/repeated recovery | Map/costmap/route | Plan/costmap/state and physical route | Cancel; clear/replan site safely | Oscillation, contact risk, repeated recovery |
| Stops before destination | Safety/localization/power | Safety, nav result, localization, power | Treat stop as valid; correct cause | Obstacle sensing appears wrong |
| Named location invalid | Registry/release/map | Inspect candidate/location/release identity | Use approved compatible record | Direct DB edit seems necessary |
| Microphone/wake/TTS fails | Audio/SavoMind/socket | Speech readiness, Edge journal, socket/device | Use non-voice control; maintainer audio check | Protocol or device crash repeats |
| UI blank/stale/touch absent | Display/input/readiness | Edge journal, device permissions, source timestamps | Use observer; repair presentation only | UI shows stale data as current or sends commands |
| Invalid battery reading | ADC/calibration/stale input | Raw status, validity, meter by maintainer | Keep motion blocked; recalibrate | Reading conflicts with physical condition |
| UPS warning/shutdown request | UPS/power | UPS/status/health and load | STOP; controlled shutdown | Swelling, heat, sag, or repeated request |

Do not begin with rebuilding, reinstalling ROS, deleting `build/install/log`,
resetting a database, editing release manifests, or disabling safety. Collect an
incident report for any safety-relevant symptom.
