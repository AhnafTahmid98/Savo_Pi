# VL53L1X ULD vendor directory

Place the ST VL53L1X Ultra Lite Driver files here before enabling
`SAVO_PERCEPTION_REQUIRE_VL53L1X_ULD`:

```text
include/VL53L1X_api.h
include/VL53L1X_types.h
include/VL53L1X_calibration.h
src/VL53L1X_api.c
src/VL53L1X_calibration.c
platform/VL53L1X_platform.h
platform/VL53L1X_platform.c
```

The platform implementation must access the Linux I2C device used by Robot Savo.
Third-party source and license files are intentionally not fabricated here; add
the authoritative ST distribution together with its license notice.
