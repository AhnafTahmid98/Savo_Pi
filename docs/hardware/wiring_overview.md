# Wiring Overview

This page is an ownership-level wiring record. Pin-to-pin cable details remain measurement-pending in the [cable map](cable_and_connector_map.md); do not infer them from this diagram.

```text
Core Pi
 +-- I2C bus 1 -- Freenove/PCA9685 0x40 -- motor outputs ch 0..7
 |                                      `-- pan/tilt servos ch 15/14
 +-- I2C bus 1 -- BNO055 0x28
 +-- I2C bus 1 -- TCA9548A 0x70 -- ch2/ch3 VL53L1X 0x29
 +-- I2C bus 1 -- ADS7830 0x48 -- base battery sense ch2
 +-- I2C bus 1 -- Core UPS 0x36
 +-- GPIO ------- four quadrature encoder pairs
 +-- GPIO 27/22 - ultrasonic trigger/echo
 +-- USB -------- RPLIDAR A1 (/dev/ttyUSB0 configured)
 +-- CSI/GStreamer Pi Camera 2 NoIR
 +-- Ethernet ---- Edge Pi

Edge Pi
 +-- USB3 ------- RealSense D435
 +-- USB/audio -- ReSpeaker and playback path
 +-- I2C bus 1 - Edge UPS 0x36
 +-- display/touch path (connector evidence pending)
```

Power and signal harnesses should be routed separately where practical, strain-relieved, keyed, labelled at both ends, and service-looped without entering LiDAR/camera fields of view or wheel envelopes. Do not share a ground or power source merely because the diagram groups a subsystem; verify the real power architecture and current capacity.

Before energizing, continuity-check every power rail and ground, confirm polarity/voltage under no-load, confirm ultrasonic echo level compatibility, check I2C pull-ups, and isolate the motor power stage. First software validation is safe-idle with motors physically unable to move the robot.
