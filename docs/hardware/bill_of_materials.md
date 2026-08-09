# Bill of Materials

This is a source-derived functional BOM. Quantity is the expected robot allocation, not a purchase or as-built confirmation. Record manufacturer part number, serial, supplier, revision, and installed quantity during hardware audit.

| Item | Expected qty | Source evidence | As-built status |
| --- | ---: | --- | --- |
| Raspberry Pi Core computer | 1 | Role/deployment and URDF | Model/RAM/serial pending |
| Raspberry Pi Edge computer | 1 | Role/deployment and URDF | Model/RAM/serial pending |
| Core and Edge UPS HAT | 2 | I2C power configs (`0x36`) | Exact model/revision/capacity pending |
| Freenove mecanum motor/PCA9685 board | 1 | Base/head configs, `0x40` | Board revision pending |
| Mecanum motor/wheel/encoder assemblies | 4 | Base/localization/URDF | Ratio/CPR/mechanical model pending |
| RPLIDAR A1 | 1 | `savo_lidar` config | Serial/revision pending |
| BNO055 IMU | 1 | Localization config, `0x28` | Board/axis mounting pending |
| TCA9548A I2C multiplexer | 1 | Perception config, `0x70` | Board revision pending |
| VL53L1X ToF sensor | 2 | Mux channels 2 and 3 | Module/optics pending |
| Front ultrasonic sensor | 1 | GPIO/config; HC-SR04 naming in records | Exact module/level interface pending |
| ADS7830 ADC | 1 | Power config, `0x48`, channel 2 | Board/revision/divider pending |
| Intel RealSense D435 | 1 | Edge config | Installed serial must match/rebind |
| Pi Camera 2 NoIR | 1 | Head camera pipeline/docs | Camera/bracket revision pending |
| Pan and tilt servos | 2 | Head config, PCA channels 15/14 | Model/horn/bracket pending |
| ReSpeaker microphone device | 1 | Speech config/docs | Exact model/USB identity pending |
| 800 x 480 display/touch assembly | 1 | UI/description | Model/interface pending |
| Base battery | 1 | Power monitor config | Chemistry/capacity/connector pending |
| Ethernet interconnect | 1 | Network deployment | Cable category/length pending |

Also inventory converters, fuse/holder, emergency power control, charger, terminal blocks, cable assemblies, brackets, decks, standoffs, fasteners, cooling, storage media, and speakers. Nothing absent from software should be assumed absent from hardware. BOM closure requires labelled photographs and serial/revision evidence.
