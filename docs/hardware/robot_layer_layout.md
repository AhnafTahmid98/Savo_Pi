# Robot Layer Layout

This is a functional placement model, not a measured assembly drawing.

```text
Top sensing/interaction
  RPLIDAR, ReSpeaker, display, pan/tilt + Pi camera
  D435 position per provisional base_link coordinates

Compute/electronics decks
  Core Pi + Freenove/motor interface + Core UPS
  Edge Pi + Edge UPS
  I2C mux, ADC, IMU, cable distribution (actual positions pending)

Base/perimeter
  four mecanum wheels/motors/encoders
  left/right ToF and front ultrasonic
  base battery and power isolation (actual mounting pending)
```

URDF places Core and Edge board centres provisionally at `[-0.045, +0.045, 0.080]` and `[-0.045, -0.045, 0.080]` metres from `base_link`. It models lower/middle/top decks but does not prove which physical component is attached to each deck.

Layer design must keep mass low and centred, allow access to battery isolation/fuses/storage media/connectors, separate noisy motor wiring from sensing/audio, preserve airflow, and avoid cable motion during head travel. The D435 and LiDAR require unobstructed fields; UPS/battery removal must not require disturbing calibrated sensors where possible.

Record each physical component's layer, x/y/z datum, bracket/fastener, connector access direction, removal sequence, mass, and photograph during the measurement campaign. Update URDF only from reviewed measurements and re-run footprint/TF/collision validation after changes.
