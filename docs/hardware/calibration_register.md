# Calibration Register

Status values: `CONFIGURED` means present in source but not measured here; `PENDING` means no authoritative value; `VALIDATED` requires dated retained evidence.

| Calibration item | Current configured baseline | Status | Required evidence |
| --- | --- | --- | --- |
| Wheel diameter | `0.065 m` | CONFIGURED | Loaded rolling-distance trials |
| Kinematic wheelbase / track | `0.165 / 0.165 m` | CONFIGURED, conflict | Measure contact centres; reconcile URDF `0.230 / 0.200 m` implication |
| Encoder CPR/decoding/gear | `20 / x4 / 1.0` | CONFIGURED | Shaft/wheel revolution counts and gearbox identification |
| Encoder polarity | all false | CONFIGURED | Each wheel forward/lateral/yaw sign test |
| Motor polarity | all inverted | CONFIGURED | Wheels-raised direction test |
| IMU mode/rate | NDOF, `25 Hz` | CONFIGURED | Axis, calibration, bias/noise, magnetic survey |
| EKF covariances | YAML values | CONFIGURED | Static/dynamic logs and residual analysis |
| Fixed sensor extrinsics | `sensor_mounts.yaml` | CONFIGURED, provisional | Datum survey plus live frame check |
| Head zero | pan `72 deg`, tilt `55 deg` | CONFIGURED | Mechanical centre, optical axis, stop clearance |
| Head limits | pan `0..170`, tilt `45..130 deg` | CONFIGURED | Servo/bracket sweep under load |
| Servo pulse range | `500..2500 us` | CONFIGURED | Model-safe pulse and mechanical stop test |
| Base ADC scale/offset | package YAML | CONFIGURED | Calibrated meter at multiple voltages/loads |
| UPS voltage/SOC | HAT telemetry | PENDING | Meter comparison and discharge curve |
| Range thresholds | front `.25/.80`, side `.08/.25 m` | CONFIGURED | Target/material/FOV and stopping-distance trials |
| D435 depth/VO | profile configuration | PENDING | USB mode, depth scale, alignment, extrinsic, trajectory comparison |
| LiDAR mount/rate | z `.205 m`, `5.5 Hz` expected | CONFIGURED | Level/alignment, live rate/range scan |

For every `VALIDATED` entry add robot hardware revision, source commit, date/operator, tool ID/calibration date, environment/load, raw artifact, calculation, accepted value/tolerance, reviewer, and configuration change reference. Calibration expiry or hardware replacement returns the item to pending.
