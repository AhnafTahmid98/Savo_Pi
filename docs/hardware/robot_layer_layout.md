# Robot Layer Layout

Robot Savo uses a four-layer physical layout. The layer design separates low-level hardware, compute units, interaction hardware, cameras, and LiDAR so the robot stays easier to wire, debug, and maintain.

Final dimensions should be updated after the printed parts are installed and measured.

## Layer summary

| Layer          | Main contents                                                          | Main role                                                   |
| -------------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- |
| Layer 0 / base | Freenove board, motors, wheels, ToF sensors, ultrasonic sensor         | Drive hardware and close-range sensing                      |
| Layer 1        | `savo-core` Pi, breadboard, speakers, display                          | Core compute, wiring hub, audio output, user-facing display |
| Layer 2        | ReSpeaker mic array, `savo-edge` Pi, RealSense camera, pan-tilt camera | Audio input, edge compute, depth/camera processing          |
| Layer 3 / top  | LiDAR only                                                             | Clear 360-degree scan area for mapping and navigation       |

## Layer 0 / base

Layer 0 is the drive and close-range sensing layer.

Main components:

* Freenove motor/control board
* motors
* mecanum wheels
* ToF sensors
* ultrasonic sensor

This layer is closest to the floor and contains the hardware directly related to movement and near-field obstacle detection.

The ToF and ultrasonic sensors are placed low enough to detect close obstacles such as feet, bags, and objects that may not be detected reliably by the top-mounted LiDAR.

## Layer 1

Layer 1 contains the main robot authority computer and user-facing hardware.

Main components:

* `savo-core` Raspberry Pi
* breadboard / wiring area
* speakers
* display

`savo-core` owns movement authority, safety decisions, localization, mapping, and navigation. Keeping `savo-core` on this layer makes it close to the base hardware while still leaving space for wiring and service access.

The display and speakers are placed here so the robot can communicate with nearby users during interaction and navigation.

## Layer 2

Layer 2 contains the edge compute and higher-level perception/audio hardware.

Main components:

* ReSpeaker mic array
* `savo-edge` Raspberry Pi
* RealSense camera
* pan-tilt camera

`savo-edge` handles heavier helper tasks such as RealSense/depth processing, visual odometry, speech/audio, UI support, and Dockerized AI/helper services from the separate `Robot_Savo_Server` repository.

The ReSpeaker mic array should have an open position for better voice capture. The RealSense and pan-tilt camera should be mounted with a clear forward view.

## Layer 3 / top

Layer 3 is reserved for the LiDAR.

Main component:

* RPLIDAR A1

The LiDAR is placed on the top layer to keep the scan area clear from robot body parts, wires, cameras, and other hardware. This improves 2D SLAM, mapping, localization, and Nav2 obstacle detection.

## Compute placement

| Compute unit | Physical layer | Main responsibility                                                    |
| ------------ | -------------- | ---------------------------------------------------------------------- |
| `savo-core`  | Layer 1        | Movement authority, control, safety, localization, mapping, navigation |
| `savo-edge`  | Layer 2        | RealSense/depth, VO, speech/audio, UI, AI/helper services              |

The two Pis will be connected directly over Ethernet.

## Sensor placement notes

| Sensor              | Expected layer | Notes                               |
| ------------------- | -------------- | ----------------------------------- |
| ToF sensors         | Layer 0        | Close-range obstacle sensing        |
| Ultrasonic sensor   | Layer 0        | Backup close-range front sensing    |
| RealSense camera    | Layer 2        | Depth and 3D obstacle support       |
| Pan-tilt camera     | Layer 2        | Camera view and future visual tasks |
| ReSpeaker mic array | Layer 2        | Voice input                         |
| LiDAR               | Layer 3        | 2D SLAM and navigation scan         |

## Measurement status

The current layer layout is fixed conceptually, but exact dimensions are not final yet.

The following values must be measured after the printed parts are installed:

* plate length and width
* plate thickness
* floor-to-layer heights
* wheel center positions
* LiDAR center position
* RealSense camera position
* pan-tilt camera position
* ToF and ultrasonic positions
* display position
* ReSpeaker position
* `savo-core` and `savo-edge` mounting positions

These measurements will be used to update:

```text
src/shared/savo_description/config/robot_dimensions.yaml
src/shared/savo_description/config/wheel_geometry.yaml
src/shared/savo_description/config/sensor_mounts.yaml
```

## Documentation media

Hardware photos and layer screenshots should be added later after the physical layout is final.

Recommended location:

```text
docs/assets/screenshots/
```

Video demonstrations can be linked from YouTube when available.
