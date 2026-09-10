# Robot Savo — typed frontier-to-navigation goal handoff status.
#
# sequence and request_id identify one selected frontier goal across all of its
# active and terminal handoff states. Consumers must not infer acknowledgement
# from observing an intermediate state.

uint32 CONTRACT_VERSION=1

uint32 contract_version
builtin_interfaces/Time stamp

bool accepted
uint64 sequence
string request_id
string state
bool active
bool terminal
string reason
string action_name
