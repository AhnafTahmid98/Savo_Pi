# Robot Savo — Navigation State
# Compact summary of what the navigation stack is doing right now.
# This will be serialized to JSON and sent to the LLM server.

std_msgs/Header header

# High-level navigation state:
#  - "IDLE"        : not moving, no active goal
#  - "NAVIGATING"  : following a path to a goal
#  - "FOLLOWING"   : following a person
#  - "MAPPING"     : mapping / exploration mode
#  - "PAUSED"      : temporarily stopped, goal still active
#  - "STOPPED"     : fully stopped, no motion commands
#  - "ERROR"       : navigation stack has a problem
string state

# Canonical ID of the active goal (for LLM + map), e.g. "A201", "Info Desk".
string goal_id

# Human-friendly label for the goal, e.g. "Room A201 (Electronics Lab)".
string goal_label

# Remaining distance along the path to the goal, in metres.
# Use a negative value (e.g. -1.0) if unknown.
float32 distance_to_goal_m

# Current commanded base velocity (same sign convention as /cmd_vel).
# vx: forward (+) / backward (-) in the robot base frame
# vy: left (+) / right (-) for mecanum strafing
# wz: yaw rotation around Z, CCW positive
float32 cmd_vx_mps
float32 cmd_vy_mps
float32 cmd_wz_radps

# Path and safety flags.
bool have_path       # true if a valid path exists
bool blocked         # true if obstacles are blocking progress
bool emergency_stop  # true if near-field safety layer has latched

# Frame used for distance_to_goal_m, usually "map".
string frame_id
