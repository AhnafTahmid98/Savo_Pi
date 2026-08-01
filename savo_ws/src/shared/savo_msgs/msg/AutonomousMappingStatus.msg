# Robot Savo — typed autonomous mapping mission status.
#
# Ownership:
# - savo_mapping owns this status and the mission lifecycle.
# - savo_nav owns execution of each admitted exploration goal.
# - savo_control and savo_perception remain final movement/safety authorities.
# - savo_head owns semantic head motion and AprilTag visual confirmation.

uint32 CONTRACT_VERSION=3

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
uint8 STATE_CAPTURING_START_POSE=14
uint8 STATE_INITIAL_SCAN360=15
uint8 STATE_INITIAL_HEAD_SCAN=16
uint8 STATE_CONDITIONAL_SCAN360=17
uint8 STATE_COVERAGE_PENDING=18
uint8 STATE_COVERAGE=19
uint8 STATE_RETURNING_TO_START=20
uint8 STATE_FINAL_SCAN360=21
uint8 STATE_FINAL_HEAD_SCAN=22
uint8 STATE_VERIFYING_LOCATIONS=23
uint8 STATE_AWAITING_APPROVAL=24
uint8 STATE_RELEASING=25

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
uint8 RESULT_SCAN_FAILED=10
uint8 RESULT_START_POSE_UNAVAILABLE=11
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

bool start_pose_capture_started
bool start_pose_capture_complete
bool start_pose_valid
geometry_msgs/PoseStamped start_pose_map
uint64 start_map_generation
string start_pose_reason

bool initial_scan360_complete
bool initial_scan360_succeeded
bool initial_head_scan_complete
bool initial_head_scan_succeeded
uint32 conditional_scan360_completed

bool scan360_started
bool scan360_active
bool scan360_complete
bool scan360_succeeded
string scan360_stage
string scan360_state
string scan360_reason

bool head_scan_started
bool head_scan_active
bool head_scan_paused
bool head_scan_complete
bool head_scan_succeeded
string head_scan_stage
string head_scan_state
string head_scan_reason

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

bool coverage_planning_started
bool coverage_planning_complete
bool coverage_plan_valid
uint64 coverage_plan_generation
uint64 coverage_map_generation
uint32 coverage_total_waypoints
bool coverage_execution_started
bool coverage_execution_active
bool coverage_execution_complete
bool coverage_execution_succeeded
string coverage_mission_id
string coverage_state
string coverage_reason
uint32 coverage_current_waypoint
uint32 coverage_completed_waypoints
float64 coverage_completion_ratio
float64 coverage_remaining_distance_m
uint32 coverage_restart_attempts

bool return_to_start_started
bool return_to_start_active
bool return_to_start_complete
bool return_to_start_succeeded
float64 return_to_start_distance_m
string return_to_start_state
string return_to_start_reason
uint32 return_to_start_attempts

bool final_scan360_complete
bool final_scan360_succeeded
bool final_head_scan_complete
bool final_head_scan_succeeded

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
