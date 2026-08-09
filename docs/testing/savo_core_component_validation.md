# Savo Core Component Validation Status

This is a non-authoritative status index. Execute the linked detailed test IDs; do not substitute a row or historical observation for current-source PASS.

| Component | Owner/interface | Required plan evidence | Current status |
| --- | --- | --- | --- |
| Core UPS and base ADC | `savo_power`; I2C `0x36` / ADS7830 `0x48` channel 2 | PWR-001–008 | `NOT RUN`; calibration-dependent |
| RPLIDAR A1 | `savo_lidar`; configured serial device | LID-001–007 | Historical baseline; current regression required |
| BNO055 | `savo_localization`; I2C bus 1, `0x28` | LOC-001–004/006–008 | Historical baseline; orientation/drift regression required |
| Four encoders | `savo_localization`; configured GPIO pairs | LOC-001–006/008 | Historical baseline; every sign/scale requires regression |
| PCA9685 and motors | `savo_base`; I2C bus 1, `0x40` | BAS-001–009 | Historical baseline; shared head initialization unresolved |
| ToF mux and sensors | `savo_perception`; TCA9548A channels/config | PER-001–006/010 | Historical baseline; identity/threshold regression required |
| Ultrasonic | `savo_perception`; configured GPIO | PER-001–006/010 | Historical baseline; threshold/stale regression required |
| Head servos/Pi Camera | `savo_head`; PCA channels 15/14 and libcamera | HED-001–007 | Historical baseline; current limits/mount/camera required |

## Safety boundary

Do not publish raw `/cmd_vel` or `/cmd_vel_safe`, write PCA duty directly, or test a legacy `/e_stop` assumption. After non-actuating checks, motion uses the bounded `BAS`, `CTL`, and `PER` procedures with E-stop, stands and explicit authorization. A wrong wheel/sensor/authority result is FAIL, not BLOCKED.

## Evidence record

Use [the result template](test_result_template.md) and link the dated evidence. No current-source Core hardware execution record was created by the Phase 7 documentation audit.
