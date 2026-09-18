# SAVO edge UI

The production UI is the active C++ framebuffer path built as `ui_node`. It
renders directly to the 800×480 edge display and never publishes motion, safety,
mapping approval, supervisor, or navigation commands.

Live read-only subscriptions cover core/edge state, robot mode, safety,
navigation, mapping, locations, speech lifecycle/transcript/response, and the
existing power feeds. Callbacks only bound and copy display text; rendering stays
on the UI timer. Independent freshness timers show `STALE` and block safety
presentation when data ages out. Camera preview remains optional and disabled in
the production profile until its bandwidth is validated.

Production startup is owned by `savo-ui-runtime.service`, installed beside
`savo_edge.service` by the role-scoped Edge installer. Distributed Edge launch
keeps `SAVO_START_UI=false`, so there is one framebuffer/UI node owner. The
standalone `savo-ui.service` exists only for display maintenance; its explicit
`--standalone` installer refuses an enabled or active Edge/production UI owner,
installs the renderer-matched paths environment, and does not enable or start the unit.

```bash
ros2 launch savo_ui ui_bringup.launch.py profile:=pc
```

The PC profile writes preview frames and does not access robot hardware.
