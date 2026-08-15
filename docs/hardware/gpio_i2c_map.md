# GPIO and I2C Map

Numbers below are BCM GPIO numbers and source-configured addresses. They are not proof of installed wiring. Core and Edge each have an independent I2C bus 1.

## Core GPIO allocation

| Function | GPIO | Direction | Owner |
| --- | ---: | --- | --- |
| Front-left encoder A / B | 20 / 21 | Input | `savo_localization` |
| Front-right encoder A / B | 13 / 25 | Input | `savo_localization` |
| Rear-left encoder A / B | 23 / 24 | Input | `savo_localization` |
| Rear-right encoder A / B | 26 / 12 | Input | `savo_localization` |
| Front ultrasonic trigger | 27 | Output | `savo_perception` |
| Front ultrasonic echo | 22 | Input | `savo_perception` |

Verify voltage levels before connection. In particular, this repository does not prove an HC-SR04 echo level-shifter/divider is installed.

## I2C allocation

| Host/bus | Address | Device | Owner / allocation |
| --- | --- | --- | --- |
| Core bus 1 | `0x28` | BNO055 IMU | `savo_localization` |
| Core bus 1 | `0x36` | Core UPS monitor | `savo_power` |
| Core bus 1 | `0x40` | Freenove PCA9685 | Motor channels 0–7; head channels 14–15 |
| Core bus 1 | `0x48` | ADS7830 | Base battery channel 2 |
| Core bus 1 | `0x70` | TCA9548A | ToF mux; left ch 2, right ch 3 |
| Core mux ch 2/3 | `0x29` | VL53L1X left/right | `savo_perception` |
| Edge bus 1 | `0x36` | Edge UPS monitor | `savo_power` |

The repeated UPS address is valid because the devices are on separate hosts. The base and head use the same PCA9685 and both drivers initialize its chip-wide PWM frequency. Channels do not overlap, but startup/reset/concurrency must be validated as one shared-device contract.

At commissioning, capture `gpioinfo`, `i2cdetect -y 1` on each host, mux-channel scans, pull-up/logic-voltage evidence, and owner/version. Keep motors disabled while probing. Never run generic I2C writes against a live motor controller.
