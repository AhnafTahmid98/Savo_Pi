uint8 STATE_UNKNOWN=0
uint8 STATE_APPROVED=1
uint8 STATE_RETIRED=2

uint8 state
bool enabled
uint64 record_revision

string location_id
string display_name
string[] aliases
string semantic_type

string map_id
uint32 map_revision
string map_release_id

geometry_msgs/PoseStamped approach_pose

bool confirmation_pose_valid
geometry_msgs/PoseStamped confirmation_pose

string tag_family
int32 tag_id

bool tag_pose_map_valid
geometry_msgs/PoseStamped tag_pose_map

bool arrival_confirmation_required

string building
string floor
string area
string notes

string source_candidate_id

builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
