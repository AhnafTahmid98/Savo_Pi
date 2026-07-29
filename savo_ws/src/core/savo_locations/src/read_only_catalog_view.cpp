// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_locations/read_only_catalog_view.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>
#include <vector>

#include "savo_locations/normalization.hpp"


namespace savo_locations
{
namespace
{

constexpr std::size_t
  kMaximumLocationIdLength{128U};


bool is_ascii_alphanumeric(
  const char character)
{
  const auto value =
    static_cast<unsigned char>(character);

  return std::isalnum(value) != 0;
}


bool is_valid_location_id(
  const std::string_view input)
{
  if (
    input.empty() ||
    input.size() >
      kMaximumLocationIdLength ||
    !is_ascii_alphanumeric(input.front()))
  {
    return false;
  }

  for (
    const char character :
    input.substr(1U))
  {
    if (
      is_ascii_alphanumeric(character) ||
      character == '.' ||
      character == '_' ||
      character == ':' ||
      character == '-')
    {
      continue;
    }

    return false;
  }

  return true;
}


bool map_matches(
  const LocationRecordData & record,
  const std::string_view map_id,
  const std::uint32_t map_revision)
{
  return
    record.location.map.map_id == map_id &&
    record.location.map.map_revision ==
      map_revision;
}


ReadResolveResult finalize_resolve(
  const LocationRecordData & record,
  const ResolveMatchType match_type,
  const std::string & normalized_query)
{
  ReadResolveResult result;

  result.normalized_query =
    normalized_query;

  result.match_type = match_type;
  result.location = record;

  if (
    record.state ==
    LocationState::kRetired)
  {
    result.code =
      ReadResolveCode::kRetired;

    result.reason =
      "location is retired";

    return result;
  }

  if (!record.enabled) {
    result.code =
      ReadResolveCode::kDisabled;

    result.reason =
      "location is disabled";

    return result;
  }

  if (
    record.state !=
    LocationState::kApproved)
  {
    result.code =
      ReadResolveCode::kNotFound;

    result.reason =
      "location is not approved";

    return result;
  }

  result.code =
    ReadResolveCode::kResolved;

  result.reason =
    "location resolved";

  return result;
}


struct IdentityMatch
{
  const LocationRecordData * record{
    nullptr};

  ResolveMatchType match_type{
    ResolveMatchType::kNone};
};

}  // namespace


ReadOnlyCatalogView::ReadOnlyCatalogView(
  CatalogSnapshot snapshot)
{
  replace(std::move(snapshot));
}


void ReadOnlyCatalogView::replace(
  CatalogSnapshot snapshot)
{
  locations_ =
    std::move(snapshot.locations);

  std::sort(
    locations_.begin(),
    locations_.end(),
    [](
      const LocationRecordData & lhs,
      const LocationRecordData & rhs)
    {
      return
        lhs.location.location_id <
        rhs.location.location_id;
    });
}


ReadResolveResult
ReadOnlyCatalogView::resolve(
  const ReadResolveRequest & request) const
{
  ReadResolveResult result;

  const std::string query =
    trim_ascii(request.query);

  result.normalized_query =
    normalize_lookup_key(query);

  if (
    query.empty() ||
    result.normalized_query.empty())
  {
    result.code =
      ReadResolveCode::kInvalidQuery;

    result.reason =
      "location query is empty";

    return result;
  }

  if (
    request.enforce_map_context &&
    (
      trim_ascii(request.map_id).empty() ||
      request.map_revision == 0U
    ))
  {
    result.code =
      ReadResolveCode::kInvalidQuery;

    result.reason =
      "enforced map context requires "
      "map_id and map_revision";

    return result;
  }

  // Exact canonical-ID precedence.
  for (
    const auto & record :
    locations_)
  {
    if (
      normalize_lookup_key(
        record.location.location_id) !=
      result.normalized_query)
    {
      continue;
    }

    if (
      request.enforce_map_context &&
      !map_matches(
        record,
        request.map_id,
        request.map_revision))
    {
      result.code =
        ReadResolveCode::kMapMismatch;

      result.reason =
        "location exists in another "
        "map context";

      result.location = record;

      return result;
    }

    return finalize_resolve(
      record,
      ResolveMatchType::kLocationId,
      result.normalized_query);
  }

  std::vector<IdentityMatch> matches;

  for (
    const auto & record :
    locations_)
  {
    ResolveMatchType match_type{
      ResolveMatchType::kNone};

    if (
      normalize_lookup_key(
        record.location.display_name) ==
      result.normalized_query)
    {
      match_type =
        ResolveMatchType::kDisplayName;
    }
    else
    {
      for (
        const auto & alias :
        record.location.aliases)
      {
        if (
          normalize_lookup_key(alias) ==
          result.normalized_query)
        {
          match_type =
            ResolveMatchType::kAlias;

          break;
        }
      }
    }

    if (
      match_type !=
      ResolveMatchType::kNone)
    {
      matches.push_back(
        IdentityMatch{
          &record,
          match_type});
    }
  }

  if (matches.empty()) {
    result.code =
      ReadResolveCode::kNotFound;

    result.reason =
      "location query did not match";

    return result;
  }

  if (request.enforce_map_context) {
    std::vector<IdentityMatch>
      scoped_matches;

    for (
      const auto & match :
      matches)
    {
      if (
        map_matches(
          *match.record,
          request.map_id,
          request.map_revision))
      {
        scoped_matches.push_back(match);
      }
    }

    if (scoped_matches.empty()) {
      result.code =
        ReadResolveCode::kMapMismatch;

      result.reason =
        "matching identity exists in "
        "another map context";

      return result;
    }

    matches =
      std::move(scoped_matches);
  }

  if (matches.size() > 1U) {
    result.code =
      ReadResolveCode::kAmbiguous;

    result.reason =
      "location query is ambiguous";

    for (
      const auto & match :
      matches)
    {
      result.ambiguous_location_ids.push_back(
        match.record
          ->location
          .location_id);
    }

    std::sort(
      result.ambiguous_location_ids.begin(),
      result.ambiguous_location_ids.end());

    result.ambiguous_location_ids.erase(
      std::unique(
        result.ambiguous_location_ids.begin(),
        result.ambiguous_location_ids.end()),
      result.ambiguous_location_ids.end());

    return result;
  }

  return finalize_resolve(
    *matches.front().record,
    matches.front().match_type,
    result.normalized_query);
}


ReadGetResult ReadOnlyCatalogView::get(
  const std::string_view location_id,
  const bool include_disabled,
  const bool include_retired) const
{
  ReadGetResult result;

  if (!is_valid_location_id(location_id)) {
    result.code =
      ReadGetCode::kInvalidId;

    result.reason =
      "location_id is not canonical";

    return result;
  }

  const auto iterator =
    std::find_if(
      locations_.begin(),
      locations_.end(),
      [location_id](
        const LocationRecordData & record)
      {
        return
          record.location.location_id ==
          location_id;
      });

  if (iterator == locations_.end()) {
    result.code =
      ReadGetCode::kNotFound;

    result.reason =
      "location was not found";

    return result;
  }

  result.location = *iterator;

  if (
    iterator->state ==
      LocationState::kRetired &&
    !include_retired)
  {
    result.code =
      ReadGetCode::kRetired;

    result.reason =
      "location is retired";

    return result;
  }

  if (
    !iterator->enabled &&
    !include_disabled)
  {
    result.code =
      ReadGetCode::kDisabled;

    result.reason =
      "location is disabled";

    return result;
  }

  result.code =
    ReadGetCode::kFound;

  result.reason =
    "location found";

  return result;
}


ReadListResult ReadOnlyCatalogView::list(
  const ReadListRequest & request) const
{
  ReadListResult result;

  if (request.state_filter > 2U) {
    result.valid = false;
    result.reason =
      "state_filter is invalid";

    return result;
  }

  if (
    request.enforce_map_context &&
    (
      trim_ascii(request.map_id).empty() ||
      request.map_revision == 0U
    ))
  {
    result.valid = false;
    result.reason =
      "enforced map context requires "
      "map_id and map_revision";

    return result;
  }

  if (
    request.map_revision != 0U &&
    trim_ascii(request.map_id).empty())
  {
    result.valid = false;
    result.reason =
      "map_revision requires map_id";

    return result;
  }

  const std::string semantic_filter =
    normalize_lookup_key(
      trim_ascii(request.semantic_type));

  for (
    const auto & record :
    locations_)
  {
    if (
      !request.map_id.empty() &&
      record.location.map.map_id !=
        request.map_id)
    {
      continue;
    }

    if (
      request.map_revision != 0U &&
      record.location.map.map_revision !=
        request.map_revision)
    {
      continue;
    }

    if (
      !semantic_filter.empty() &&
      normalize_lookup_key(
        record.location.semantic_type) !=
        semantic_filter)
    {
      continue;
    }

    if (
      request.state_filter == 1U &&
      record.state !=
        LocationState::kApproved)
    {
      continue;
    }

    if (
      request.state_filter == 2U &&
      record.state !=
        LocationState::kRetired)
    {
      continue;
    }

    if (
      request.enabled_only &&
      !record.enabled)
    {
      continue;
    }

    result.locations.push_back(record);
  }

  result.reason =
    "locations listed";

  return result;
}


const std::vector<LocationRecordData> &
ReadOnlyCatalogView::locations() const noexcept
{
  return locations_;
}

}  // namespace savo_locations
