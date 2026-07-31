# Robot Savo — typed frontier-planning observation.
#
# Published by savo_mapping/frontier_explorer_node. This message reports the
# most recent planner evaluation separately from the explorer's transient
# runtime state so completion detection never depends on parsing JSON text.

uint32 CONTRACT_VERSION=1

uint8 EXHAUSTION_NONE=0
uint8 EXHAUSTION_NO_FRONTIERS=1
uint8 EXHAUSTION_NO_REACHABLE_FRONTIERS=2
uint8 EXHAUSTION_NO_SELECTABLE_FRONTIER=3

uint32 contract_version
builtin_interfaces/Time stamp

string state
string reason

bool configured_enabled
bool runtime_authority_received
bool runtime_enabled
bool enabled
bool map_received

uint64 map_generation
uint64 planned_map_generation
uint64 plan_sequence

string planning_status
string planning_reason
uint8 exhaustion_kind

string handoff_state
bool goal_pending
string last_goal_id

uint32 detected_frontiers
uint32 reachable_frontiers
