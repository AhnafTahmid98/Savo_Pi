# Base test plan

Prerequisites: locked physical geometry, completed wiring inspection, charged
base, functional E-stop, two operators, wheels raised on rated stands, and
control mode confirmed STOP. Required hardware is the complete drivetrain,
encoder set, base power monitor, stands, and E-stop.

Exact command: first run `python3 tools/diag/motion/odom_test.py`. A designated
operator may then run the approved control package’s bounded test interface with
its explicit `--allow-motion` option; this repository diagnostic never publishes
velocity directly.

Expected result: wheels follow the documented mecanum direction, odometry sign
and scale are plausible, releasing the command returns gated velocity to zero,
and STOP remains available. Failure result: wrong wheel/direction, encoder loss,
unexpected movement, nonzero safe velocity, or power fault is FAIL, never PASS.

Abort condition: any person enters the exclusion zone, the chassis moves on its
stands, temperature/current exceeds the hardware limit, telemetry goes stale, or
STOP is not immediate. Cleanup: engage E-stop, set STOP, remove motor power, and
verify `/cmd_vel_safe` is zero. Record commands, JSON diagnostics, currents,
encoder plots, operator names, timestamps, video, and PASS/FAIL/BLOCKED.
