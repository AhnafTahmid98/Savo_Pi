#ifndef SAVO_LOCATIONS__MODEL_HPP_
#define SAVO_LOCATIONS__MODEL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "savo_locations/types.hpp"

namespace savo_locations
{

struct PoseData
{
  std::string frame_id{"map"};

  double x{0.0};
  double y{0.0};
  double z{0.0};

  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};


struct MapContext
{
  std::string map_id;
  std::uint32_t map_revision{0U};
  std::string map_release_id;
};


struct TagBinding
{
  std::string family;
  std::int32_t id{-1};
};


struct LocationDraft
{
  std::string location_id;
  std::string display_name;
  std::vector<std::string> aliases;
  std::string semantic_type;

  MapContext map;

  PoseData approach_pose;
  std::optional<PoseData> confirmation_pose;
  std::optional<PoseData> tag_pose_map;

  TagBinding tag;

  bool arrival_confirmation_required{true};

  std::string building;
  std::string floor;
  std::string area;
  std::string notes;
};


struct LocationRecordData
{
  LocationState state{LocationState::kApproved};
  bool enabled{true};
  std::uint64_t record_revision{1U};

  LocationDraft location;

  std::string source_candidate_id;
};


struct CandidateDraft
{
  std::string candidate_id;

  MapContext map;
  TagBinding tag;
  PoseData tag_pose_map;

  double detection_quality{0.0};
  std::uint32_t accepted_observations{0U};
  double position_stddev_m{0.0};
  double yaw_stddev_rad{0.0};

  std::optional<PoseData> approach_pose;
  std::optional<PoseData> confirmation_pose;

  std::string suggested_location_id;
  std::string suggested_display_name;
  std::vector<std::string> suggested_aliases;
  std::string suggested_semantic_type;

  std::string building;
  std::string floor;
  std::string area;
  std::string notes;

  std::string source_session_id;
  std::string source_component;
};


struct CandidateRecordData
{
  CandidateState state{
    CandidateState::kPendingReview};

  std::uint64_t candidate_revision{1U};

  CandidateDraft candidate;

  std::string review_reason;
  std::string approved_location_id;
};


struct ApprovalRequest
{
  std::string candidate_id;
  std::uint64_t expected_candidate_revision{0U};

  std::string location_id;
  std::string display_name;
  std::vector<std::string> aliases;
  std::string semantic_type;

  std::optional<PoseData> approach_pose;
  std::optional<PoseData> confirmation_pose;

  bool arrival_confirmation_required{true};

  std::string building;
  std::string floor;
  std::string area;
  std::string notes;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__MODEL_HPP_
