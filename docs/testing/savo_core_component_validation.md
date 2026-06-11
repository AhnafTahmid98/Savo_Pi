# savo-core Component Validation

Standalone hardware validation for every physical component on `savo-core` before ROS 2 integration.

Follow the process in [`component_validation_overview.md`](component_validation_overview.md).

## Validation table

| Component              | Interface      | Standalone test                                     | Expected result                                      | Status      |
| ---------------------- | -------------- | --------------------------------------------------- | ---------------------------------------------------- | ----------- |
| UPS HAT                | I2C-1 `0x36`  | `i2cdetect -y 1`, `bat.py`                          | `0x36` visible; voltage and capacity printed         | Not started |
| RPLIDAR A1             | USB serial     | `ros2 launch rplidar_ros rplidar_a1_launch.py`      | `/scan` topic publishes; `ros2 topic hz /scan` ≈ 10 Hz | Not started |
| BNO055 IMU             | I2C-1 `0x28`  | `tools/diag/sensors/imu_test.py`                    | CHIP_ID=0xA0; Euler/accel/gyro data streams          | Not started |
| 4 wheel encoders       | GPIO (lgpio)   | `tools/diag/motion/encoders_test.py`                | All 4 encoders count correctly in both directions    | Not started |
| PCA9685 / motor driver | I2C-1 `0x40`  | `i2cdetect -y 1`; manual PWM pulse test             | `0x40` visible; wheels respond to PWM duty cycle     | Not started |
| DC motors              | Via PCA9685    | Manual PWM sweep via motor driver test script       | All 4 wheels spin in correct direction at commanded speed | Not started |
| VL53L1X ToF sensors    | I2C-1 (multi-addr) | `i2cdetect -y 1`; standalone range read script  | Each sensor returns valid range in mm                | Not started |
| Ultrasonic sensor      | GPIO trigger/echo | `tools/diag/sensors/` ultrasonic script or CLI   | Distance reading updates at expected rate            | Not started |
| Safety stop pipeline   | ROS 2 topic    | Publish `True` on `/e_stop`; observe motor halt     | Motor output stops immediately; resumes on clear     | Not started |
| Base driver            | ROS 2 node     | `ros2 topic pub /cmd_vel` → observe wheel motion    | Wheels move at correct speed; odometry publishes     | Not started |

## Notes

- Validate in the order listed: power first, then sensors, then actuators, then ROS integration.
- Motor tests require a safe bench setup (wheels off the ground or robot blocked).
- ToF sensors share the I2C bus; address conflicts must be resolved before testing multiple units together.
- Safety stop pipeline test requires the base driver to be running.
