// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "savo_msgs/msg/location_candidate.hpp"
#include "savo_msgs/msg/location_record.hpp"
#include "savo_msgs/srv/get_location_candidate.hpp"
#include "savo_msgs/srv/list_location_candidates.hpp"
#include "savo_msgs/srv/review_location_candidate.hpp"

namespace savo_mapping
{
namespace
{

using Json = nlohmann::json;
using Candidate = savo_msgs::msg::LocationCandidate;
using Location = savo_msgs::msg::LocationRecord;
using GetCandidate = savo_msgs::srv::GetLocationCandidate;
using ListCandidates = savo_msgs::srv::ListLocationCandidates;
using ReviewCandidate = savo_msgs::srv::ReviewLocationCandidate;

constexpr int kExitSuccess = 0;
constexpr int kExitUsage = 2;
constexpr int kExitDependencyUnavailable = 3;
constexpr int kExitNotFound = 4;
constexpr int kExitOperationRejected = 5;
constexpr int kExitTimedOut = 6;

struct CliError : public std::runtime_error
{
  CliError(const int exit_code, const std::string & message)
  : std::runtime_error(message), code(exit_code)
  {
  }

  int code;
};

enum class Command
{
  kHelp,
  kList,
  kShow,
  kApprove,
  kReject,
};

struct Options
{
  Command command{Command::kHelp};
  bool json{false};
  double timeout_s{5.0};

  std::string list_service{"/savo_locations/candidates/list"};
  std::string candidate_service{"/savo_locations/candidates/get"};
  std::string review_service{"/savo_mapping/locations/review"};

  std::uint8_t state_filter{
    ListCandidates::Request::STATE_FILTER_PENDING};
  bool enforce_map_context{false};
  std::string map_id{};
  std::uint32_t map_revision{0U};

  std::string candidate_id{};
  std::uint64_t expected_revision{0U};
  bool revision_supplied{false};
  std::string request_id{};
  std::string actor_id{};

  std::string location_id{};
  std::string display_name{};
  std::vector<std::string> aliases{};
  std::string semantic_type{};
  bool arrival_confirmation_required{true};
  std::string building{};
  std::string floor{};
  std::string area{};
  std::string notes{};
  std::string rejection_reason{};
};

[[nodiscard]] bool blank(const std::string_view value)
{
  for (const char character : value) {
    if (
      character != ' ' && character != '\t' && character != '\n' &&
      character != '\r' && character != '\f' && character != '\v')
    {
      return false;
    }
  }

  return true;
}

[[nodiscard]] std::string require_value(
  const std::vector<std::string> & arguments,
  std::size_t * index,
  const std::string_view option)
{
  if (index == nullptr || *index + 1U >= arguments.size()) {
    throw CliError(
            kExitUsage,
            std::string{option} + " requires a value");
  }

  ++(*index);
  return arguments[*index];
}

template<typename IntegerT>
[[nodiscard]] IntegerT parse_unsigned(
  const std::string_view value,
  const std::string_view label)
{
  static_assert(std::is_unsigned_v<IntegerT>);

  if (value.empty()) {
    throw CliError(
            kExitUsage,
            std::string{label} + " is required");
  }

  std::uint64_t parsed = 0U;
  const auto result = std::from_chars(
    value.data(),
    value.data() + value.size(),
    parsed);

  if (
    result.ec != std::errc{} ||
    result.ptr != value.data() + value.size() ||
    parsed > static_cast<std::uint64_t>(
      std::numeric_limits<IntegerT>::max()))
  {
    throw CliError(
            kExitUsage,
            std::string{label} + " must be an unsigned integer");
  }

  return static_cast<IntegerT>(parsed);
}

[[nodiscard]] double parse_timeout(const std::string & value)
{
  std::size_t consumed = 0U;
  double parsed = 0.0;

  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw CliError(kExitUsage, "timeout must be a positive number");
  }

  if (
    consumed != value.size() || !std::isfinite(parsed) || parsed <= 0.0)
  {
    throw CliError(kExitUsage, "timeout must be a positive number");
  }

  return parsed;
}

[[nodiscard]] bool parse_bool(
  const std::string & value,
  const std::string_view label)
{
  if (value == "true" || value == "1" || value == "yes") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    return false;
  }

  throw CliError(
          kExitUsage,
          std::string{label} + " must be true or false");
}

[[nodiscard]] std::uint8_t parse_state_filter(const std::string & value)
{
  if (value == "all") {
    return ListCandidates::Request::STATE_FILTER_ALL;
  }
  if (value == "pending") {
    return ListCandidates::Request::STATE_FILTER_PENDING;
  }
  if (value == "approved") {
    return ListCandidates::Request::STATE_FILTER_APPROVED;
  }
  if (value == "rejected") {
    return ListCandidates::Request::STATE_FILTER_REJECTED;
  }

  throw CliError(
          kExitUsage,
          "state must be all, pending, approved or rejected");
}

[[nodiscard]] std::string generated_request_id()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return "location-review-cli-" + std::to_string(nanoseconds);
}

void print_usage(std::ostream & output)
{
  output <<
    "Robot Savo location candidate review CLI\n\n"
    "Usage:\n"
    "  location_review_cli list [options]\n"
    "  location_review_cli show CANDIDATE_ID [options]\n"
    "  location_review_cli inspect CANDIDATE_ID [options]\n"
    "  location_review_cli approve CANDIDATE_ID --actor ACTOR [options]\n"
    "  location_review_cli reject CANDIDATE_ID --actor ACTOR --reason TEXT "
    "[options]\n\n"
    "Common options:\n"
    "  --json                         Emit one JSON object\n"
    "  --timeout SEC                  Dependency and response timeout\n"
    "  --candidate-service NAME       Read-only candidate lookup service\n"
    "  --candidate-list-service NAME  Read-only candidate list service\n"
    "  --review-service NAME          Authorized mapping review service\n\n"
    "List options:\n"
    "  --state all|pending|approved|rejected\n"
    "  --map-id ID --map-revision REV\n\n"
    "Review options:\n"
    "  --revision REV                 Expected revision; otherwise looked up\n"
    "  --request-id ID                Audit request identifier\n"
    "  --actor ACTOR                  Required operator identity\n"
    "  --location-id ID               Approval override\n"
    "  --display-name NAME            Approval override\n"
    "  --alias ALIAS                  Repeatable approval alias\n"
    "  --semantic-type TYPE           Approval override\n"
    "  --building VALUE --floor VALUE --area VALUE --notes VALUE\n"
    "  --arrival-confirmation-required true|false\n"
    "  --reason TEXT                  Required rejection reason\n\n"
    "Exit codes:\n"
    "  0 success; 2 usage; 3 dependency unavailable; 4 not found;\n"
    "  5 review rejected; 6 timed out\n";
}

[[nodiscard]] Options parse_options(const int argc, char ** argv)
{
  std::vector<std::string> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int index = 0; index < argc; ++index) {
    arguments.emplace_back(argv[index] == nullptr ? "" : argv[index]);
  }

  Options options;

  if (arguments.size() < 2U) {
    return options;
  }

  const auto & command = arguments[1];
  if (command == "help" || command == "--help" || command == "-h") {
    return options;
  }
  if (command == "list") {
    options.command = Command::kList;
  } else if (command == "show" || command == "inspect") {
    options.command = Command::kShow;
  } else if (command == "approve") {
    options.command = Command::kApprove;
  } else if (command == "reject") {
    options.command = Command::kReject;
  } else {
    throw CliError(kExitUsage, "unknown command: " + command);
  }

  for (std::size_t index = 2U; index < arguments.size(); ++index) {
    const auto & token = arguments[index];

    if (token == "--help" || token == "-h") {
      options.command = Command::kHelp;
      return options;
    } else if (token == "--json") {
      options.json = true;
    } else if (token == "--timeout") {
      options.timeout_s = parse_timeout(
        require_value(arguments, &index, token));
    } else if (token == "--candidate-service") {
      options.candidate_service = require_value(arguments, &index, token);
    } else if (token == "--candidate-list-service") {
      options.list_service = require_value(arguments, &index, token);
    } else if (token == "--review-service") {
      options.review_service = require_value(arguments, &index, token);
    } else if (token == "--state") {
      options.state_filter = parse_state_filter(
        require_value(arguments, &index, token));
    } else if (token == "--map-id") {
      options.map_id = require_value(arguments, &index, token);
    } else if (token == "--map-revision") {
      options.map_revision = parse_unsigned<std::uint32_t>(
        require_value(arguments, &index, token),
        "map revision");
    } else if (token == "--revision") {
      options.expected_revision = parse_unsigned<std::uint64_t>(
        require_value(arguments, &index, token),
        "candidate revision");
      options.revision_supplied = true;
    } else if (token == "--request-id") {
      options.request_id = require_value(arguments, &index, token);
    } else if (token == "--actor") {
      options.actor_id = require_value(arguments, &index, token);
    } else if (token == "--location-id") {
      options.location_id = require_value(arguments, &index, token);
    } else if (token == "--display-name") {
      options.display_name = require_value(arguments, &index, token);
    } else if (token == "--alias") {
      options.aliases.push_back(require_value(arguments, &index, token));
    } else if (token == "--semantic-type") {
      options.semantic_type = require_value(arguments, &index, token);
    } else if (token == "--building") {
      options.building = require_value(arguments, &index, token);
    } else if (token == "--floor") {
      options.floor = require_value(arguments, &index, token);
    } else if (token == "--area") {
      options.area = require_value(arguments, &index, token);
    } else if (token == "--notes") {
      options.notes = require_value(arguments, &index, token);
    } else if (token == "--arrival-confirmation-required") {
      options.arrival_confirmation_required = parse_bool(
        require_value(arguments, &index, token),
        "arrival confirmation");
    } else if (token == "--reason") {
      options.rejection_reason = require_value(arguments, &index, token);
    } else if (!token.empty() && token.front() == '-') {
      throw CliError(kExitUsage, "unknown option: " + token);
    } else if (options.candidate_id.empty()) {
      options.candidate_id = token;
    } else {
      throw CliError(kExitUsage, "unexpected argument: " + token);
    }
  }

  if (
    options.list_service.empty() || options.candidate_service.empty() ||
    options.review_service.empty())
  {
    throw CliError(kExitUsage, "service names cannot be empty");
  }

  if (options.command == Command::kList) {
    if (!options.candidate_id.empty()) {
      throw CliError(
              kExitUsage,
              "list does not accept a candidate ID");
    }

    const bool has_map_id = !blank(options.map_id);
    const bool has_map_revision = options.map_revision != 0U;
    if (has_map_id != has_map_revision) {
      throw CliError(
              kExitUsage,
              "--map-id and --map-revision must be supplied together");
    }
    options.enforce_map_context = has_map_id;
    return options;
  }

  if (blank(options.candidate_id)) {
    throw CliError(kExitUsage, "candidate ID is required");
  }

  if (options.command == Command::kShow) {
    return options;
  }

  if (blank(options.actor_id)) {
    throw CliError(kExitUsage, "--actor is required for review operations");
  }

  if (options.revision_supplied && options.expected_revision == 0U) {
    throw CliError(kExitUsage, "candidate revision must be non-zero");
  }

  if (
    options.command == Command::kReject &&
    blank(options.rejection_reason))
  {
    throw CliError(kExitUsage, "--reason is required for rejection");
  }

  if (blank(options.request_id)) {
    options.request_id = generated_request_id();
  }

  return options;
}

[[nodiscard]] std::string candidate_state_text(const std::uint8_t state)
{
  switch (state) {
    case Candidate::STATE_PENDING_REVIEW:
      return "pending";
    case Candidate::STATE_APPROVED:
      return "approved";
    case Candidate::STATE_REJECTED:
      return "rejected";
    default:
      return "unknown";
  }
}

[[nodiscard]] Json pose_json(
  const geometry_msgs::msg::PoseStamped & pose)
{
  return Json{
    {"frame_id", pose.header.frame_id},
    {"x", pose.pose.position.x},
    {"y", pose.pose.position.y},
    {"z", pose.pose.position.z},
    {"qx", pose.pose.orientation.x},
    {"qy", pose.pose.orientation.y},
    {"qz", pose.pose.orientation.z},
    {"qw", pose.pose.orientation.w},
  };
}

[[nodiscard]] Json candidate_json(const Candidate & candidate)
{
  Json output{
    {"candidate_id", candidate.candidate_id},
    {"candidate_revision", candidate.candidate_revision},
    {"state", candidate_state_text(candidate.state)},
    {"state_code", candidate.state},
    {"map_id", candidate.map_id},
    {"map_revision", candidate.map_revision},
    {"map_release_id", candidate.map_release_id},
    {"tag_family", candidate.tag_family},
    {"tag_id", candidate.tag_id},
    {"suggested_location_id", candidate.suggested_location_id},
    {"suggested_display_name", candidate.suggested_display_name},
    {"suggested_aliases", candidate.suggested_aliases},
    {"suggested_semantic_type", candidate.suggested_semantic_type},
    {"building", candidate.building},
    {"floor", candidate.floor},
    {"area", candidate.area},
    {"notes", candidate.notes},
    {"review_reason", candidate.review_reason},
    {"approach_pose_valid", candidate.approach_pose_valid},
    {"confirmation_pose_valid", candidate.confirmation_pose_valid},
  };

  output["tag_pose_map"] = pose_json(candidate.tag_pose_map);
  if (candidate.approach_pose_valid) {
    output["approach_pose"] = pose_json(candidate.approach_pose);
  }
  if (candidate.confirmation_pose_valid) {
    output["confirmation_pose"] = pose_json(candidate.confirmation_pose);
  }

  return output;
}

[[nodiscard]] Json location_json(const Location & location)
{
  return Json{
    {"location_id", location.location_id},
    {"display_name", location.display_name},
    {"aliases", location.aliases},
    {"semantic_type", location.semantic_type},
    {"record_revision", location.record_revision},
    {"enabled", location.enabled},
    {"map_id", location.map_id},
    {"map_revision", location.map_revision},
    {"source_candidate_id", location.source_candidate_id},
  };
}

void print_candidate_human(const Candidate & candidate)
{
  std::cout << "Candidate: " << candidate.candidate_id << '\n'
            << "State: " << candidate_state_text(candidate.state) << '\n'
            << "Revision: " << candidate.candidate_revision << '\n'
            << "Map: " << candidate.map_id << " r"
            << candidate.map_revision << '\n'
            << "AprilTag: " << candidate.tag_family << ':'
            << candidate.tag_id << '\n'
            << "Suggested location: " << candidate.suggested_location_id
            << " — " << candidate.suggested_display_name << '\n'
            << "Semantic type: " << candidate.suggested_semantic_type << '\n'
            << "Building/floor/area: " << candidate.building << " / "
            << candidate.floor << " / " << candidate.area << '\n'
            << "Approach pose: "
            << (candidate.approach_pose_valid ? "valid" : "missing") << '\n'
            << "Confirmation pose: "
            << (candidate.confirmation_pose_valid ? "valid" : "missing")
            << '\n';

  if (!candidate.review_reason.empty()) {
    std::cout << "Review reason: " << candidate.review_reason << '\n';
  }
}

void print_list_human(const std::vector<Candidate> & candidates)
{
  if (candidates.empty()) {
    std::cout << "No matching location candidates.\n";
    return;
  }

  std::cout << std::left
            << std::setw(28) << "CANDIDATE"
            << std::setw(11) << "STATE"
            << std::setw(10) << "REVISION"
            << std::setw(22) << "MAP"
            << std::setw(18) << "TAG"
            << "SUGGESTED LOCATION\n";

  for (const auto & candidate : candidates) {
    std::ostringstream map;
    map << candidate.map_id << " r" << candidate.map_revision;
    std::ostringstream tag;
    tag << candidate.tag_family << ':' << candidate.tag_id;

    std::cout << std::left
              << std::setw(28) << candidate.candidate_id
              << std::setw(11) << candidate_state_text(candidate.state)
              << std::setw(10) << candidate.candidate_revision
              << std::setw(22) << map.str()
              << std::setw(18) << tag.str()
              << candidate.suggested_location_id << " — "
              << candidate.suggested_display_name << '\n';
  }
}

[[nodiscard]] std::chrono::nanoseconds timeout_duration(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

template<typename ServiceT>
[[nodiscard]] std::shared_ptr<typename ServiceT::Response> call_service(
  rclcpp::executors::SingleThreadedExecutor * executor,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::shared_ptr<typename ServiceT::Request> & request,
  const double timeout_s,
  const std::string_view dependency)
{
  if (executor == nullptr) {
    throw CliError(kExitOperationRejected, "executor is unavailable");
  }

  if (!client->wait_for_service(timeout_duration(timeout_s))) {
    throw CliError(
            kExitDependencyUnavailable,
            std::string{dependency} + " service unavailable");
  }

  auto future = client->async_send_request(request);
  const auto result = executor->spin_until_future_complete(
    future,
    timeout_duration(timeout_s));

  if (result == rclcpp::FutureReturnCode::TIMEOUT) {
    throw CliError(
            kExitTimedOut,
            std::string{dependency} + " service timed out");
  }
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    throw CliError(
            kExitOperationRejected,
            std::string{dependency} + " service call interrupted");
  }

  return future.get();
}

class LocationReviewCli
{
public:
  explicit LocationReviewCli(const Options & options)
  : options_(options),
    node_(std::make_shared<rclcpp::Node>("location_review_cli"))
  {
    list_client_ = node_->create_client<ListCandidates>(options_.list_service);
    candidate_client_ =
      node_->create_client<GetCandidate>(options_.candidate_service);
    review_client_ =
      node_->create_client<ReviewCandidate>(options_.review_service);
    executor_.add_node(node_);
  }

  ~LocationReviewCli()
  {
    executor_.remove_node(node_);
  }

  int run()
  {
    switch (options_.command) {
      case Command::kList:
        return list();
      case Command::kShow:
        return show();
      case Command::kApprove:
        return review(true);
      case Command::kReject:
        return review(false);
      case Command::kHelp:
        print_usage(std::cout);
        return kExitSuccess;
    }

    throw CliError(kExitUsage, "unsupported command");
  }

private:
  int list()
  {
    auto request = std::make_shared<ListCandidates::Request>();
    request->state_filter = options_.state_filter;
    request->enforce_map_context = options_.enforce_map_context;
    request->map_id = options_.map_id;
    request->map_revision = options_.map_revision;

    const auto response = call_service<ListCandidates>(
      &executor_,
      list_client_,
      request,
      options_.timeout_s,
      "candidate list");

    if (!response->success) {
      throw CliError(kExitOperationRejected, response->reason);
    }

    if (options_.json) {
      Json candidates = Json::array();
      for (const auto & candidate : response->candidates) {
        candidates.push_back(candidate_json(candidate));
      }
      std::cout << Json{
        {"schema_version", 1},
        {"command", "list"},
        {"success", true},
        {"count", response->candidates.size()},
        {"reason", response->reason},
        {"candidates", std::move(candidates)},
        }.dump() << '\n';
    } else {
      print_list_human(response->candidates);
    }

    return kExitSuccess;
  }

  [[nodiscard]] Candidate load_candidate()
  {
    auto request = std::make_shared<GetCandidate::Request>();
    request->candidate_id = options_.candidate_id;

    const auto response = call_service<GetCandidate>(
      &executor_,
      candidate_client_,
      request,
      options_.timeout_s,
      "candidate lookup");

    if (!response->found) {
      const int exit_code =
        response->result_code == GetCandidate::Response::RESULT_NOT_FOUND ?
        kExitNotFound : kExitOperationRejected;
      throw CliError(exit_code, response->reason);
    }

    return response->candidate;
  }

  int show()
  {
    const auto candidate = load_candidate();

    if (options_.json) {
      std::cout << Json{
        {"schema_version", 1},
        {"command", "show"},
        {"success", true},
        {"candidate", candidate_json(candidate)},
        }.dump() << '\n';
    } else {
      print_candidate_human(candidate);
    }

    return kExitSuccess;
  }

  int review(const bool approve)
  {
    std::uint64_t revision = options_.expected_revision;

    if (!options_.revision_supplied) {
      const auto candidate = load_candidate();
      revision = candidate.candidate_revision;
    }

    auto request = std::make_shared<ReviewCandidate::Request>();
    request->request_id = options_.request_id;
    request->actor_id = options_.actor_id;
    request->decision = approve ?
      ReviewCandidate::Request::DECISION_APPROVE :
      ReviewCandidate::Request::DECISION_REJECT;
    request->candidate_id = options_.candidate_id;
    request->expected_candidate_revision = revision;
    request->location_id = options_.location_id;
    request->display_name = options_.display_name;
    request->aliases = options_.aliases;
    request->semantic_type = options_.semantic_type;
    request->arrival_confirmation_required =
      options_.arrival_confirmation_required;
    request->building = options_.building;
    request->floor = options_.floor;
    request->area = options_.area;
    request->notes = options_.notes;
    request->rejection_reason = options_.rejection_reason;

    const auto response = call_service<ReviewCandidate>(
      &executor_,
      review_client_,
      request,
      options_.timeout_s,
      "authorized review");

    if (options_.json) {
      Json output{
        {"schema_version", 1},
        {"command", approve ? "approve" : "reject"},
        {"request_id", options_.request_id},
        {"candidate_id", options_.candidate_id},
        {"expected_candidate_revision", revision},
        {"completed", response->completed},
        {"approved", response->approved},
        {"rejected", response->rejected},
        {"result_code", response->result_code},
        {"reason", response->reason},
        {"candidate", candidate_json(response->candidate)},
      };
      if (response->approved) {
        output["location"] = location_json(response->location);
      }
      std::cout << output.dump() << '\n';
    } else if (response->approved) {
      std::cout << "APPROVED candidate=" << options_.candidate_id
                << " location=" << response->location.location_id
                << " revision=" << response->location.record_revision
                << " reason=" << response->reason << '\n';
    } else if (response->rejected) {
      std::cout << "REJECTED candidate=" << options_.candidate_id
                << " revision=" << response->candidate.candidate_revision
                << " reason=" << response->reason << '\n';
    } else {
      std::cerr << "Review failed: " << response->reason
                << " (result_code="
                << static_cast<unsigned int>(response->result_code)
                << ")\n";
    }

    if (response->completed) {
      return kExitSuccess;
    }
    if (
      response->result_code ==
      ReviewCandidate::Response::RESULT_CANDIDATE_NOT_FOUND)
    {
      return kExitNotFound;
    }
    if (
      response->result_code ==
      ReviewCandidate::Response::RESULT_SUPERVISOR_UNAVAILABLE ||
      response->result_code ==
      ReviewCandidate::Response::RESULT_REGISTRY_UNAVAILABLE)
    {
      return kExitDependencyUnavailable;
    }
    if (
      response->result_code ==
      ReviewCandidate::Response::RESULT_TIMED_OUT)
    {
      return kExitTimedOut;
    }
    return kExitOperationRejected;
  }

  Options options_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<ListCandidates>::SharedPtr list_client_;
  rclcpp::Client<GetCandidate>::SharedPtr candidate_client_;
  rclcpp::Client<ReviewCandidate>::SharedPtr review_client_;
};

}  // namespace
}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    const auto options = savo_mapping::parse_options(argc, argv);
    savo_mapping::LocationReviewCli cli{options};
    exit_code = cli.run();
  } catch (const savo_mapping::CliError & error) {
    std::cerr << "Error: " << error.what() << '\n';
    if (error.code == savo_mapping::kExitUsage) {
      savo_mapping::print_usage(std::cerr);
    }
    exit_code = error.code;
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    exit_code = savo_mapping::kExitOperationRejected;
  }

  rclcpp::shutdown();
  return exit_code;
}
