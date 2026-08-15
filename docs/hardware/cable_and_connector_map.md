# Cable and Connector Map

No verified pin-to-pin harness record exists in the inspected source. This register therefore separates known logical endpoints from physical details that must be traced on the robot.

| Cable ID | Endpoint A | Endpoint B | Known contract | Physical details/status |
| --- | --- | --- | --- | --- |
| NET-01 | Core Ethernet | Edge Ethernet | Direct `192.168.50.0/24` link | Connector/length/routing pending |
| USB-01 | Core USB | RPLIDAR A1 | `/dev/ttyUSB0`, 115200 configured | Port/adapter/retention pending |
| USB-02 | Edge USB3 | RealSense D435 | RGB-D bandwidth required | Port/cable/strain relief pending |
| AUD-01 | Edge USB/audio | ReSpeaker | Speech capture | USB ID/port pending |
| CAM-01 | Core CSI path | Pi Camera 2 NoIR | GStreamer `libcamerasrc` | Connector/orientation/retention pending |
| I2C-C | Core SDA/SCL | PCA, IMU, mux, ADC, UPS | Bus 1 addresses in GPIO map | Topology/connectors/pull-ups pending |
| I2C-E | Edge SDA/SCL | Edge UPS | Bus 1, `0x36` | Connector/pull-ups pending |
| TOF-L/R | TCA9548A ch 2/3 | VL53L1X left/right | Each `0x29` | Connector/pinout/routing pending |
| US-01 | Core GPIO 27/22 | Front ultrasonic | Trigger/echo | Supply and echo level conversion pending |
| ENC-FL | Core GPIO 20/21 | FL encoder | A/B | Voltage/pinout/connector pending |
| ENC-FR | Core GPIO 13/25 | FR encoder | A/B | Voltage/pinout/connector pending |
| ENC-RL | Core GPIO 23/24 | RL encoder | A/B | Voltage/pinout/connector pending |
| ENC-RR | Core GPIO 26/12 | RR encoder | A/B | Voltage/pinout/connector pending |
| MOT-01..04 | Freenove board | Four motors | PCA channel pairs per GPIO/I2C map | Polarity/connectors/wire gauge pending |
| SRV-P/T | PCA ch 15/14 | Pan/tilt servos | 50 Hz configured | Connector polarity/current pending |
| PWR-* | Battery/UPS/converters | Loads | Three monitored domains | Full power-tree trace pending |

Trace with power removed. Record connector family, pin numbering/photo, wire colour **and** electrical function, voltage/current, gauge, length, shield/ground, fuse source, strain relief, routing layer, labels, mating-cycle/service notes, and continuity result. Wire colour alone is never authority.
