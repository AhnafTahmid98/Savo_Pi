# Safety Architecture

Robot Savo uses layered, fail-closed safeguards; none is represented as a certified safety system.

## Layers

1. Bringup rejects invalid role/mode/profile combinations and blocks motion-capable non-bench profiles without acceptable geometry.
2. Supervisor starts unarmed, owns permission/fault latch, and can revoke authority.
3. Control starts in `STOP`, admits one command lane, shapes velocity, and times out stale input.
4. Core perception fuses left/right VL53L1X and front ultrasonic range state, publishes stop/slowdown, and gates `/cmd_vel` into `/cmd_vel_safe`.
5. Base consumes only the safe command, applies its independent watchdog, and writes the drivetrain.
6. Deployment and operator procedures require staged, wheels-raised validation and physical emergency-stop access.

The optional D435 obstacle cloud supplements navigation costmaps only after explicit validation. It does not replace the Core near-field gate. UI/observer warnings are presentation, not enforcement.

## Safety state and failures

Core safety outputs include `/safety/stop`, `/safety/slowdown_factor`, and `/savo_perception/safety_state`. Missing or stale required sensor data is configured to fail safe. Configured diagnostic thresholds include front stop/slow distances of `0.25/0.80 m`, side stop/slow distances of `0.08/0.25 m`, and a `0.30 s` stale timeout; these values require physical stopping-distance validation and must not be treated as certified limits.

Loss of supervisor authority, required readiness, fresh command, or safety input must stop or block the operation. A fault may require explicit operator recovery. Loss of Edge cannot make Core less restrictive.

## Outside this architecture

Map/location approval, navigation planning, and SavoMind reasoning do not determine electrical or physical safety. Software watchdogs do not replace a hardware power disconnect. Current documents do not assert a wired emergency-stop circuit because no verified circuit record exists.

Source validators confirm topology and fail-closed defaults. Required hardware evidence includes sensor coverage/false-negative tests, timing under load, stopping distance on representative surfaces, motor-output quench, emergency power removal, brownout behavior, power thresholds, and recovery after every injected failure.
