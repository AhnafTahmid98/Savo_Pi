# Power Architecture

## Monitored domains

Robot Savo models three power domains: Core UPS, Edge UPS, and the base/drivetrain battery. Each Pi reads its own UPS at I2C bus 1 address `0x36`; Core reads base voltage through ADS7830 `0x48`, channel 2. `savo_power` publishes per-domain status and Core aggregate health/shutdown request.

The physical upstream topology, battery chemistry/capacity, fuse ratings, wire gauges, converters, switch, grounding, and charging interlocks are not proven by current source and remain measurement/inspection items. Software monitoring is not a substitute for over-current, undervoltage, reverse-polarity, or emergency isolation hardware.

## Configured thresholds

| Domain | Configured value | Status |
| --- | ---: | --- |
| UPS low / critical | `3.40 / 3.20 V` | Software default; calibrate per HAT |
| Base empty / low / full | `6.40 / 7.20 / 8.40 V` | Software mapping; chemistry must be verified |
| Base low / full SOC | `20 / 95%` | Derived display/control thresholds |
| Sample/publish rate | `1 Hz` | Configured |
| Stale timeout | `5 s` | Configured |

Automatic shutdown execution is disabled in package configuration. The monitor may request shutdown; privileged deployment owns actual OS poweroff policy. A low-power event must not create a new motion authority, and shutdown must first converge on stopped/unauthorized state.

Validate ADC scale/offset against a calibrated meter at several loads, UPS telemetry against each board, sag during motor stall/start, brownout behavior, charger isolation, ground noise, fuse protection, cable heating, runtime, low/critical transitions, stale telemetry, and controlled shutdown/restart. Record calibrated coefficients and equipment in the [calibration register](calibration_register.md).
