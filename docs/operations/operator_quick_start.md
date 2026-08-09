# Operator Quick Start

This is a trained-operator checklist, not a commissioning procedure.

> [!WARNING]
> Selecting an approved manual, mapping, or navigation operation can move the
> robot. Keep the physical stop control accessible and the operating area clear.

## Start-of-session checklist

- [ ] Complete the [pre-operation inspection](pre_operation_inspection.md).
- [ ] Power Core and Edge using the approved installed power controls.
- [ ] On Core, verify `savo_core.service` is active.
- [ ] Confirm `/savo_bringup/core/state` reports the intended role/profile and
  does not report `blocked`.
- [ ] Confirm `/savo_control/mode_state` is `STOP`.
- [ ] On Edge, verify `savo_edge.service` is active.
- [ ] Confirm `/savo_bringup/edge/state` is current for required Edge features.
- [ ] Confirm safety, localization, power, TF, and applicable map readiness.
- [ ] Confirm optional bridge, speech, and UI show current—not stale—state.
- [ ] Select only a released operation covered by its specific runbook.
- [ ] At session end, cancel, return to `STOP`, and perform controlled shutdown.

Useful local checks after sourcing the installed workspace:

```bash
systemctl status savo_core.service --no-pager
ros2 topic echo --once /savo_bringup/core/state
ros2 topic echo --once /savo_control/mode_state
ros2 topic echo --once /safety/stop
ros2 topic echo --once /savo_power/health
```

Run Edge service/state checks on Edge with `savo_edge.service` and
`/savo_bringup/edge/state`.

## Do not operate if

- geometry is not locked for the requested motion profile;
- the physical emergency power/stop control is unavailable or unverified;
- startup control state is not `STOP`;
- required safety sensing, readiness, power, localization, or TF is stale;
- a navigation map release or named-location map association is unverified;
- an unexpected duplicate node, service, or command publisher exists;
- a fault, shutdown request, abnormal heat/noise/smell, loose mount, exposed
  wiring, or drivetrain obstruction is present; or
- cancellation and STOP cannot be demonstrated before entering the area.

Record date, operator, robot/release identity, selected mode/profile, inspection
result, and any deviation. A `BLOCKED` result is not a pass.

