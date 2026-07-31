#ifndef SAVO_LOCATIONS__SERVICE_NAMES_HPP_
#define SAVO_LOCATIONS__SERVICE_NAMES_HPP_

#include <string_view>

namespace savo_locations::service_names
{

inline constexpr std::string_view kResolve{
  "/savo_locations/resolve"};

inline constexpr std::string_view kGet{
  "/savo_locations/get"};

inline constexpr std::string_view kGetCandidate{
  "/savo_locations/candidates/get"};

inline constexpr std::string_view kListCandidates{
  "/savo_locations/candidates/list"};

inline constexpr std::string_view kList{
  "/savo_locations/list"};

inline constexpr std::string_view kRegisterCandidate{
  "/savo_locations/candidates/register"};

inline constexpr std::string_view kApproveCandidate{
  "/savo_locations/candidates/approve"};

inline constexpr std::string_view kRejectCandidate{
  "/savo_locations/candidates/reject"};

inline constexpr std::string_view kSetEnabled{
  "/savo_locations/set_enabled"};

inline constexpr std::string_view kRecoverStorage{
  "/savo_locations/storage/recover"};

}  // namespace savo_locations::service_names

#endif  // SAVO_LOCATIONS__SERVICE_NAMES_HPP_
