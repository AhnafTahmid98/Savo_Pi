uint8 EVENT_UNKNOWN=0
uint8 EVENT_CANDIDATE_REGISTERED=1
uint8 EVENT_LOCATION_APPROVED=2
uint8 EVENT_CANDIDATE_REJECTED=3
uint8 EVENT_LOCATION_UPDATED=4
uint8 EVENT_LOCATION_ENABLED=5
uint8 EVENT_LOCATION_DISABLED=6
uint8 EVENT_LOCATION_RETIRED=7
uint8 EVENT_IMPORT_COMPLETED=8
uint8 EVENT_STORAGE_DEGRADED=9

uint64 event_sequence
builtin_interfaces/Time stamp
uint8 event_type

string candidate_id
string location_id
uint64 entity_revision

string actor_id
string reason
