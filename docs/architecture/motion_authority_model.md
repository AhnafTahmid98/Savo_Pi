# Motion Authority Model

This model separates permission, command selection, safety intervention, and physical execution.

## Production chain

```text
manual / mapping / navigation / recovery source
                    |
                    v
            savo_supervisor permission
                    |
                    v
        savo_control mode and lane mux
                    |
             /cmd_vel_mux
                    v
      shaping and configured limits
                    |
                /cmd_vel
                    v
   savo_perception command safety gate
                    |
            /cmd_vel_safe
                    v
       savo_base watchdog and mixer
                    |
          PCA9685 -> four motors
```

`savo_supervisor` owns permission and fault/system state. The active operation owner executes and revalidates permission. `savo_control` admits one of `STOP`, `MANUAL`, `AUTO`, `NAV`, or `RECOVERY`, selects the corresponding command lane, and applies configured shaping. `savo_perception` can zero or scale the command. `savo_base` is the sole final hardware writer.

## Bounds and fail-closed behavior

Current control configuration caps shaped velocity at `0.20 m/s` linear x, `0.18 m/s` linear y, and `0.55 rad/s` angular z; it uses a `0.35 s` input timeout. Base consumes only `/cmd_vel_safe`, uses a `0.30 s` watchdog, runs at `30 Hz`, and limits duty to `3500`. These are configured safeguards, not validated safe physical limits.

An external stop forces control `STOP`. Stale commands, `/safety/stop`, invalid safety state, supervisor revocation, cancellation, and base watchdog expiry must converge on zero output. Reconnection or process restart does not implicitly re-arm. Stuck-recovery auto-request is disabled in current configuration.

## Explicitly forbidden paths

Edge, SavoMind, bridge, UI, observer, mapping, navigation, and supervisor cannot write motor channels or `/cmd_vel_safe` directly. Navigation and mapping may supply bounded lanes only through control. Operator approval cannot be inferred by a language model or presentation surface.

Validation requires source-contract tests plus wheels-raised checks for startup STOP, each command lane, timeout, stop/slowdown, authority revoke, process/link loss, motor polarity, duty limiting, and recovery cancellation. Until those pass on current hardware and geometry is locked, this chain is implemented but not production-motion authorized.
