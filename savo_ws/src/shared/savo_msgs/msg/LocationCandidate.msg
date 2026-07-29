uint8 STATE_UNKNOWN=0
uint8 STATE_PENDING_REVIEW=1
uint8 STATE_APPROVED=2
uint8 STATE_REJECTED=3

uint8 state
uint64 candidate_revision
string candidate_id

string map_id
uint32 map_revision
string map_release_id

string tag_family
int32 tag_id
geometry_msgs/PoseStamped tag_pose_map

float32 detection_quality
uint32 accepted_observations
float32 position_stddev_m
float32 yaw_stddev_rad

bool approach_pose_valid
geometry_msgs/PoseStamped approach_pose

bool confirmation_pose_valid
geometry_msgs/PoseStamped confirmation_pose

string suggested_location_id
string suggested_display_name
string[] suggested_aliases
string suggested_semantic_type

string building
string floor
string area
string notes

string source_session_id
string source_component
string review_reason

builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
