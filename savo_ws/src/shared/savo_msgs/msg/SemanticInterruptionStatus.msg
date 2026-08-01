# Robot Savo — mapping-owned semantic interruption lifecycle.
#
# Detector evidence comes from savo_head. Candidate persistence remains owned
# by savo_locations through the RegisterMappedLocation action.

uint32 CONTRACT_VERSION=1

uint8 STATE_IDLE=0
uint8 STATE_TAG_DETECTED=1
uint8 STATE_PAUSING=2
uint8 STATE_WAITING_FOR_MISSION_PAUSED=3
uint8 STATE_WAITING_FOR_SEMANTICS=4
uint8 STATE_REGISTERING=5
uint8 STATE_RESUMING=6
uint8 STATE_COMPLETED=7
uint8 STATE_FAILED=8

uint32 contract_version
std_msgs/Header header
uint8 state
string state_text
string reason
bool active

string mission_id
string map_id
uint32 map_revision

string tag_family
int32 tag_id
uint64 observation_sequence
uint64 duplicate_observations_suppressed
savo_msgs/AprilTagObservation detector_observation

string candidate_id
savo_msgs/LocationCandidate location_candidate

bool semantic_submission_received
bool registration_started
bool registration_complete
bool resume_requested
bool resume_complete
