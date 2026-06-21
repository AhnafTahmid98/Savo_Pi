# savo_supervisor

savo_supervisor will be the high-level readiness, mode, and confidence manager for Robot Savo.

This package will not directly control motors. Its job is to decide whether the robot system is ready for a requested action, such as navigation, follow mode, mapping, or interaction. Final movement authority will remain inside the safety/control stack.

Purpose

Robot Savo has many ROS packages running across two Raspberry Pi units:

savo-core: base, control, LiDAR, localization, perception, safety, navigation
savo-edge: RealSense, visual odometry, speech, UI, AI helper services

The supervisor will watch the system and answer:

Which packages/nodes should be running?
Which topics must be fresh?
Is the robot safe to move?
Is navigation allowed right now?
Is mapping allowed right now?
Is the robot only in idle/interaction mode?
What is the reason if something is blocked?

Location and Runtime

Package path:

savo_ws/src/shared/savo_supervisor

Runtime target:

savo-core

The package is placed under shared because it observes both savo-core and savo-edge, but the main supervisor should run on savo-core because savo-core owns movement authority.

Ownership Boundary

savo_supervisor owns:

robot readiness state
robot mode state
health/confidence summary
high-level action permission
status reporting for UI, app, and LLM
startup/readiness checks

savo_supervisor does not own:

motor control
low-level safety gate
emergency stop logic
Nav2 planning
SLAM/mapping logic
sensor drivers

Those remain in their own packages.

Planned Robot Modes

The supervisor will manage high-level modes such as:

Mode Meaning
BOOTING Robot backend is starting
IDLE Backend is ready, robot is not moving
INTERACT Robot is talking/listening, movement blocked
NAVIGATE Navigation command is active
FOLLOW Follow mode is active
MAPPING Mapping/SLAM mode is active
ERROR Required system is unhealthy
ESTOP Emergency stop or hard safety stop is active

Important rule:

System running does not mean robot is allowed to move.

Planned Readiness Checks

The supervisor will calculate readiness states such as:

safety_ready
localization_ready
navigation_ready
mapping_ready
speech_ready
edge_ready
base_ready
can_move

Example navigation readiness:

navigation_ready =
  base_ok
  control_ok
  safety_ok
  lidar_ok
  localization_ok
  map_ok
  battery_ok

If one required system is missing or stale, navigation will be blocked and the supervisor will publish the reason.

Planned Inputs

The supervisor may subscribe to topics such as:

/realsense/status
/lidar/status
/localization/status
/savo_base/base_state
/safety/stop
/safety/slowdown_factor
/cmd_vel_safe
/savo_intent/intent_result
/robot_status

Additional edge/core health topics may be added later.

Planned Outputs

The supervisor may publish:

/savo_supervisor/state
/savo_supervisor/mode
/savo_supervisor/readiness
/savo_supervisor/events

Example state output:

{
  "mode": "IDLE",
  "navigation_ready": true,
  "mapping_ready": true,
  "safety_ready": true,
  "can_move": false,
  "reason": "Waiting for user command"
}

Example blocked state:

{
  "mode": "ERROR",
  "navigation_ready": false,
  "can_move": false,
  "reason": "Localization stale"
}

Planned Services

Possible services:

/savo_supervisor/set_mode
/savo_supervisor/request_navigation
/savo_supervisor/request_mapping
/savo_supervisor/clear_error

These services will not directly drive the robot. They will approve or reject high-level actions based on readiness and safety state.

Systemd and Launch Relationship

systemd will start the backend services at boot.

Launch files will start ROS packages.

savo_supervisor will check whether the started system is healthy and ready.

Expected production flow:

Power on robot
-> systemd starts core/edge backend
-> launch files start packages
-> savo_supervisor checks readiness
-> robot waits in IDLE/INTERACT
-> user gives command
-> supervisor allows or blocks action
-> safety/control stack remains final movement authority

Language Plan

First version:

Python

Reason:

supervisor is orchestration/status logic
not high-rate motor/safety control
easier to develop and debug

Future hybrid version is possible if needed, but low-level safety and motion timing will remain in C++ packages such as savo_control, savo_perception, and savo_base.

Development Plan

Planned stages:

Create package skeleton under src/shared/savo_supervisor
Add supervisor.yaml with required topics and timeout thresholds
Add supervisor_node.py
Watch required topics and node health
Publish /savo_supervisor/state
Add mode model: BOOTING, IDLE, INTERACT, NAVIGATE, FOLLOW, MAPPING, ERROR, ESTOP
Add readiness logic for navigation/mapping/follow
Connect state to UI and LLM status replies
Add tests for readiness and mode transitions
Integrate with final savo_bringup and systemd services

Current Status

This package is planned but not implemented yet.

It will be created after the core ownership packages are stable:

savo_realsense
savo_lidar
savo_localization
savo_perception
savo_control
savo_base

Pointcloud/voxel readiness will be added later when savo_nav is implemented.

Later, when you want to create it quickly:

mkdir -p ~/Savo_Pi/savo_ws/src/shared/savo_supervisor
nano ~/Savo_Pi/savo_ws/src/shared/savo_supervisor/README.md
