# Savo Edge Component Validation Status

This is a status index. Execute the detailed plans and retain revision-bound evidence; device screenshots alone do not establish current package or integration PASS.

| Component | Owner/interface | Required plan evidence | Current status |
| --- | --- | --- | --- |
| Edge UPS | `savo_power`; I2C bus 1, `0x36` | PWR-001–008 | Historical device evidence; current calibration/fault regression required |
| RealSense D435 | `savo_realsense`; USB3/bound serial | RLS-001–007 | `NOT RUN` for current source |
| Visual odometry | `savo_vo`; D435 RGB-D input | VO-001–009 using `vo_bringup.launch.py` | `NOT RUN`; Core fusion remains off |
| ReSpeaker and speaker | `savo_speech`; configured ALSA devices | SPH-001–010 | `NOT RUN`; repository ALSA alias deployment gap remains |
| Display/touch | `savo_ui`; framebuffer/input | UI-001–007 | `NOT RUN` on current target |
| Typed SavoMind boundary | `savo_bridge` and speech socket | BRD-001–009, SPH-003/004/006/009 | Source/protocol baseline; target ownership/integration required |
| Edge service graph | `savo_bringup`/systemd | BRG-005/008–010 | UI companion-unit ownership decision unresolved |

STT/LLM/TTS inference belongs to external SavoMind, not an Edge ROS component. Generic Docker health is therefore not a substitute for the typed bridge/speech protocol tests.

## Historical media

Existing UPS/fan screenshots under `docs/assets/hardware` are contextual historical evidence only unless a test record supplies date, revision, configuration and reviewer. They are not current-source `HARDWARE_PASS`.

## Evidence record

Use [the result template](test_result_template.md). No physical Edge device or service was started by the Phase 7 documentation audit.
