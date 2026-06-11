# savo-edge Component Validation

Standalone hardware validation for every physical component on `savo-edge` before ROS 2 integration.

Follow the process in [`component_validation_overview.md`](component_validation_overview.md).

## Validation table

| Component                  | Interface         | Standalone test                                               | Expected result                                                 | Status      |
| -------------------------- | ----------------- | ------------------------------------------------------------- | --------------------------------------------------------------- | ----------- |
| UPS HAT                    | I2C-1 `0x36`      | `i2cdetect -y 1`, `bat.py`                                    | `0x36` visible; voltage and capacity printed                    | Not started |
| RealSense D435             | USB 3.0           | `rs-enumerate-devices`, `realsense-viewer`                    | Device listed; colour and depth streams visible in viewer       | Not started |
| ReSpeaker mic array        | USB               | `arecord -l`, `arecord -D plughw:<N>,0 -f S16_LE -r 16000 test.wav` | Device listed; recording captures audio without clipping | Not started |
| Speaker / audio output     | USB or 3.5 mm     | `aplay -l`, `aplay test.wav`                                  | Device listed; audio plays back clearly                         | Not started |
| STT (speech-to-text)       | Mic + model       | Run STT pipeline standalone; speak a phrase                   | Transcription output matches spoken phrase                      | Not started |
| TTS (text-to-speech)       | Speaker + model   | Run TTS pipeline standalone; pass test sentence               | Synthesised speech plays through speaker                        | Not started |
| Visual odometry            | RealSense D435    | `ros2 launch savo_vo savo_vo.launch.py`; move camera slowly  | `/vo/odom` or equivalent topic publishes; pose increments with motion | Not started |
| Docker Robot Savo Server   | Network / Docker  | `docker compose up`; check service health endpoints           | All containers start; health endpoints return 200               | Not started |
| UI / display (if used)     | HDMI / USB-C DP   | Boot to desktop or launch UI node                             | Display renders without artefacts; touch or input responds      | Not started |

## Notes

- Validate audio input and output separately before testing STT/TTS end-to-end.
- RealSense must pass `rs-enumerate-devices` before VO testing begins.
- Docker Robot Savo Server requires network access; confirm Tailscale or LAN connectivity first.
- Visual odometry validation uses slow manual movement to avoid tracking loss during initial test.
