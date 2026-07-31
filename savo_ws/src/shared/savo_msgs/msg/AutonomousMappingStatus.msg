# Robot Savo — typed autonomous mapping mission status.
#
# Ownership:
# - savo_mapping owns this status and the mission lifecycle.
# - savo_nav owns execution of each admitted exploration goal.
# - savo_control and savo_perception remain final movement/safety authorities.

uint32 CONTRACT_VERSION=1

uint8 STRATEGY_NONE=0
uint8 STRATEGY_FRONTIER=1
uint8 STRATEGY_COVERAGE=2

uint8 STATE_IDLE=0
uint8 STATE_STARTING=1
uint8 STATE_WAITING_FOR_AUTHORITY=2
uint8 STATE_EXPLORING=3
uint8 STATE_PAUSING=4
uint8 STATE_PAUSED=5
uint8 STATE_RESUMING=6
uint8 STATE_CANCELING=7
uint8 STATE_SAVING=8
uint8 STATE_VERIFYING=9
uint8 STATE_COMPLETED=10
uint8 STATE_CANCELED=11
uint8 STATE_FAILED=12
uint8 STATE_COMPLETION_PENDING=13

uint8 RESULT_SUCCEEDED=0
uint8 RESULT_INVALID_REQUEST=1
uint8 RESULT_BUSY=2
uint8 RESULT_READINESS_LOST=3
uint8 RESULT_NAVIGATION_FAILED=4
uint8 RESULT_TIMED_OUT=5
uint8 RESULT_CANCELED=6
uint8 RESULT_SAVE_FAILED=7
uint8 RESULT_QUALITY_REJECTED=8
uint8 RESULT_INTERNAL_ERROR=9
uint8 RESULT_NOT_TERMINAL=255

uint32 contract_version
builtin_interfaces/Time stamp

string mission_id
string actor_id
string map_id
uint32 map_revision
uint8 strategy
uint8 state
string state_text
uint8 result_code

bool active
bool runtime_authorized
bool mapping_ready
bool safety_stop_active
bool handoff_active
bool auto_save
bool require_quality_approval

uint32 goals_succeeded
uint32 goals_failed

bool frontier_status_received
bool frontier_status_fresh
string frontier_planning_status
uint64 frontier_plan_sequence
uint64 frontier_map_generation
uint32 detected_frontiers
uint32 reachable_frontiers
uint32 exhaustion_observations
float64 exhaustion_stable_duration_s
bool completion_candidate
bool completion_confirmed
string completion_reason

bool map_save_started
bool map_save_complete
bool map_saved
string map_save_reason
string saved_session_directory

bool verification_started
bool verification_complete
bool map_verified
string verification_reason

string reason
