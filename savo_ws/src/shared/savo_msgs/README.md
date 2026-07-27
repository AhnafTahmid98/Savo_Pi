# savo_msgs

Shared ROS 2 interfaces for Robot Savo.

## AprilTag contracts

### `AprilTagObservation.msg`

Represents one camera-derived AprilTag observation.

It is visual evidence only. It does not create a semantic location, approve a
location or declare that navigation succeeded.

### `ConfirmAprilTag.action`

Defines the two `savo_head` AprilTag confirmation duties:

1. `REGISTER_LOCATION`

   Confirms a newly discovered tag during mapping. `savo_mapping` can then
   georeference it, the operator can name and classify the location, and
   `savo_locations` can store the candidate or approved record.

2. `CONFIRM_ARRIVAL`

   Confirms the expected saved tag after `savo_nav` reaches the destination
   approach or confirmation pose.

## Ownership

- `savo_head`: visual detection and confirmation.
- `savo_mapping`: map-frame validation and mapping context.
- Operator app or CLI: location naming and semantic classification.
- `savo_locations`: persistent semantic-location storage.
- `savo_nav`: navigation and final arrival decision.
