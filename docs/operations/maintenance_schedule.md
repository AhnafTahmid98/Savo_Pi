# Maintenance Schedule

Intervals are operational triggers, not manufacturer claims. Shorten them for
high-use, dusty, transported, or recently repaired robots. Perform work with
motion disabled and record the hardware/software identity.

| Item | Frequency/trigger | Procedure | Evidence | Retest impact |
| --- | --- | --- | --- | --- |
| Wheels and rollers | Before each session | Inspect wear, debris, looseness, free movement | Checklist/photos | Direction and low-speed motion if disturbed |
| Fasteners/chassis | Before session; after transport/impact | Inspect decks, brackets, damage | Marked checklist | Geometry/footprint if moved |
| LiDAR/sensor clearance | Before each session | Clean safely; check mount/FOV | Photo and health state | Scan/range and TF if moved |
| Pan/tilt and cables | Before each session | Inspect sweep clearance, strain, backlash | Inspection record | Servo limits/head TF if changed |
| Wiring/connectors | Before session; periodic detailed review | Inspect strain, abrasion, heat, retention | Cable-map update | Device/component tests |
| Battery/UPS | Before session; monthly trend review | Inspect condition; compare raw readings and runtime | Voltage/UPS log | Power calibration/fault tests |
| Cooling/overheating | Before session; monthly | Inspect airflow, dust, abnormal heat | Temperature/inspection log | Load and safe-idle test |
| State backup | After approved release; before maintenance/update | Run controlled backup and verify hashes | Archive/hash/metadata | Restore drill under maintenance plan |
| Disk/log usage | Weekly or several sessions | `df`; review rotation and failed journals | Usage snapshot | None unless storage changed |
| Map/location integrity | Monthly; before navigation campaign | Verify active contract, release hashes, SQLite | Integrity report | Named navigation/map activation |
| Role software | After each release | Clean target role build/test and staged update | Commit/build/test logs | Applicable regression suite |
| Wheel geometry/odometry | After wheel/drivetrain work or drift | Measure/calibrate per register | Measurement/calibration record | Manual motion, odom, localization, navigation |
| IMU alignment | After mount change or pose anomaly | Recheck axes/interference/calibration | Sensor logs | EKF/localization |
| Safety sensors | After mount/replacement or abnormal stop | Revalidate offsets, coverage, stale/fail-safe | Range/stopping evidence | Safety gate and guarded motion |
| Servo centre/range | After head work | Calibrate centre, limits, cable clearance | Calibration record | Head TF/Scan360/arrival confirmation |
| ADC/UPS calibration | After power hardware work or drift | Compare with calibrated meter | Tool/results/coefficients | Low/critical/shutdown tests |
| Camera/depth geometry | After camera/bracket work | Measure extrinsics and validate depth/cloud | Geometry digest and logs | TF, VO, self-filter, voxel profile |
| Full geometry/footprint | Any structural change | Measure, review, regenerate, lock profile | Reviewed profile/digest | Full motion/localization/navigation acceptance |
| Incident-affected item | After any incident | Inspect/correct under incident plan | Incident and corrective action | Safety-review-selected regression |

Before formal acceptance, complete the [hardware measurement checklist](../hardware/measurement_checklist.md),
[calibration register](../hardware/calibration_register.md), role builds, and
[real-robot acceptance checklist](../testing/real_robot_acceptance_checklist.md).
Hardware replacement returns affected calibration entries to pending until
reviewed evidence closes them.
