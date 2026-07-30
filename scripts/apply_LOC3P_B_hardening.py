#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import os
import py_compile
import shutil
import tarfile
import textwrap
from datetime import datetime
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path.home() / "Savo_Pi"
WS = ROOT / "savo_ws"
SRC = WS / "src"
SCRIPTS = ROOT / "scripts"
BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"


def find_package(name: str) -> Path:
    matches: list[Path] = []
    for package_xml in SRC.rglob("package.xml"):
        try:
            root = ET.parse(package_xml).getroot()
        except (ET.ParseError, OSError):
            continue
        if root.findtext("name") == name:
            matches.append(package_xml.parent)
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {name} package, found {len(matches)}: {matches}"
        )
    return matches[0]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one match, found {count}")
    return text.replace(old, new, 1)


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def update_xml_version(path: Path, expected: str, new: str) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    node = root.find("version")
    if node is None or node.text != expected:
        raise RuntimeError(
            f"{path}: expected version {expected}, found {None if node is None else node.text}"
        )
    node.text = new
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    if not SRC.is_dir():
        raise RuntimeError(f"workspace source directory missing: {SRC}")

    SCRIPTS.mkdir(parents=True, exist_ok=True)
    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    msgs = find_package("savo_msgs")
    locations = find_package("savo_locations")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = BACKUPS / f"pre_LOC3P_B_hardening_{stamp}.tar.gz"
    manifest = LOGS / f"LOC3P_B_hardening_{stamp}.sha256"

    with tarfile.open(backup, "w:gz") as tar:
        tar.add(msgs, arcname=str(msgs.relative_to(SRC)))
        tar.add(locations, arcname=str(locations.relative_to(SRC)))

    # ------------------------------------------------------------------
    # savo_msgs: typed verified storage-recovery service.
    # ------------------------------------------------------------------
    update_xml_version(msgs / "package.xml", "0.3.0", "0.4.0")

    apriltag_test_path = msgs / "test" / "test_apriltag_interfaces.py"
    apriltag_test = apriltag_test_path.read_text(encoding="utf-8")
    apriltag_test = replace_once(
        apriltag_test,
        '    assert root.findtext("version") == "0.3.0"',
        '    assert root.findtext("version") == "0.4.0"',
        "historical AprilTag interface version contract",
    )
    write(apriltag_test_path, apriltag_test)

    recover_srv = textwrap.dedent(
        """\
        # Explicit recovery of the persistent semantic-location registry.
        #
        # Recovery is allowed only when write services are enabled. The server
        # must open a new SQLite connection, validate schema and integrity,
        # bootstrap a complete temporary catalog, and only then replace the
        # active repository and restore write readiness.

        string actor_id
        string reason
        ---
        uint8 RESULT_RECOVERED=0
        uint8 RESULT_INVALID_REQUEST=1
        uint8 RESULT_NOT_ENABLED=2
        uint8 RESULT_STORAGE_UNAVAILABLE=3
        uint8 RESULT_INTEGRITY_FAILED=4
        uint8 RESULT_INTERNAL_ERROR=5

        bool recovered
        uint8 result_code
        string reason

        uint32 schema_version
        uint64 location_count
        uint64 candidate_count
        uint64 event_count
        uint64 last_event_sequence
        """
    )
    write(msgs / "srv" / "RecoverLocationStorage.srv", recover_srv)

    cmake_path = msgs / "CMakeLists.txt"
    cmake = cmake_path.read_text(encoding="utf-8")
    cmake = replace_once(
        cmake,
        '  "srv/SetLocationEnabled.srv"\n)',
        '  "srv/SetLocationEnabled.srv"\n  "srv/RecoverLocationStorage.srv"\n)',
        "savo_msgs service registration",
    )
    write(cmake_path, cmake)

    test_interfaces_path = msgs / "test" / "test_location_interfaces.py"
    test_interfaces = test_interfaces_path.read_text(encoding="utf-8")
    test_interfaces = replace_once(
        test_interfaces,
        '        "srv/SetLocationEnabled.srv",\n',
        '        "srv/SetLocationEnabled.srv",\n'
        '        "srv/RecoverLocationStorage.srv",\n',
        "savo_msgs interface contract",
    )
    test_interfaces += textwrap.dedent(
        """

        def test_storage_recovery_contract() -> None:
            text = read("srv/RecoverLocationStorage.srv")

            for token in (
                "string actor_id",
                "RESULT_RECOVERED=0",
                "RESULT_NOT_ENABLED=2",
                "RESULT_INTEGRITY_FAILED=4",
                "uint64 last_event_sequence",
            ):
                assert token in text
        """
    )
    write(test_interfaces_path, test_interfaces)

    # ------------------------------------------------------------------
    # savo_locations: explicit verified recovery and runtime hardening.
    # ------------------------------------------------------------------
    update_xml_version(locations / "package.xml", "0.10.0", "0.11.0")

    locations_cmake_path = locations / "CMakeLists.txt"
    locations_cmake = locations_cmake_path.read_text(encoding="utf-8")
    locations_cmake = replace_once(
        locations_cmake,
        "project(savo_locations VERSION 0.10.0 LANGUAGES CXX)",
        "project(savo_locations VERSION 0.11.0 LANGUAGES CXX)",
        "savo_locations CMake version",
    )
    write(locations_cmake_path, locations_cmake)

    constants_path = locations / "include" / "savo_locations" / "constants.hpp"
    constants = constants_path.read_text(encoding="utf-8")
    constants = replace_once(
        constants,
        '  "0.10.0"};',
        '  "0.11.0"};',
        "savo_locations constant version",
    )
    write(constants_path, constants)

    service_names_path = locations / "include" / "savo_locations" / "service_names.hpp"
    service_names = service_names_path.read_text(encoding="utf-8")
    service_names = replace_once(
        service_names,
        'inline constexpr std::string_view kSetEnabled{\n  "/savo_locations/set_enabled"};\n',
        'inline constexpr std::string_view kSetEnabled{\n'
        '  "/savo_locations/set_enabled"};\n\n'
        'inline constexpr std::string_view kRecoverStorage{\n'
        '  "/savo_locations/storage/recover"};\n',
        "storage recovery service name",
    )
    write(service_names_path, service_names)

    header_path = locations / "include" / "savo_locations" / "location_registry_node.hpp"
    header = header_path.read_text(encoding="utf-8")
    header = replace_once(
        header,
        '#include "savo_msgs/srv/register_location_candidate.hpp"\n',
        '#include "savo_msgs/srv/register_location_candidate.hpp"\n'
        '#include "savo_msgs/srv/recover_location_storage.hpp"\n',
        "recovery service include",
    )
    header = replace_once(
        header,
        '  using SetEnabledService =\n    savo_msgs::srv::SetLocationEnabled;\n',
        '  using SetEnabledService =\n'
        '    savo_msgs::srv::SetLocationEnabled;\n\n'
        '  using RecoveryService =\n'
        '    savo_msgs::srv::RecoverLocationStorage;\n',
        "recovery service alias",
    )
    header = replace_once(
        header,
        '  void handle_set_enabled(\n    const std::shared_ptr<\n      SetEnabledService::Request> request,\n    std::shared_ptr<\n      SetEnabledService::Response> response);\n',
        '  void handle_set_enabled(\n'
        '    const std::shared_ptr<\n'
        '      SetEnabledService::Request> request,\n'
        '    std::shared_ptr<\n'
        '      SetEnabledService::Response> response);\n\n'
        '  void handle_recover_storage(\n'
        '    const std::shared_ptr<\n'
        '      RecoveryService::Request> request,\n'
        '    std::shared_ptr<\n'
        '      RecoveryService::Response> response);\n',
        "recovery callback declaration",
    )
    header = replace_once(
        header,
        '  rclcpp::Service<SetEnabledService>::SharedPtr\n    set_enabled_service_;\n',
        '  rclcpp::Service<SetEnabledService>::SharedPtr\n'
        '    set_enabled_service_;\n\n'
        '  rclcpp::Service<RecoveryService>::SharedPtr\n'
        '    recovery_service_;\n',
        "recovery service member",
    )
    write(header_path, header)

    source_path = locations / "src" / "location_registry_node.cpp"
    source = source_path.read_text(encoding="utf-8")
    source = replace_once(
        source,
        '  set_enabled_service_ =\n    create_service<SetEnabledService>(\n      std::string(\n        ::savo_locations::service_names::kSetEnabled),\n      std::bind(\n        &LocationRegistryNode::handle_set_enabled,\n        this,\n        std::placeholders::_1,\n        std::placeholders::_2));\n\n  initialize_storage();\n',
        '  set_enabled_service_ =\n'
        '    create_service<SetEnabledService>(\n'
        '      std::string(\n'
        '        ::savo_locations::service_names::kSetEnabled),\n'
        '      std::bind(\n'
        '        &LocationRegistryNode::handle_set_enabled,\n'
        '        this,\n'
        '        std::placeholders::_1,\n'
        '        std::placeholders::_2));\n\n'
        '  recovery_service_ =\n'
        '    create_service<RecoveryService>(\n'
        '      std::string(\n'
        '        ::savo_locations::service_names::kRecoverStorage),\n'
        '      std::bind(\n'
        '        &LocationRegistryNode::handle_recover_storage,\n'
        '        this,\n'
        '        std::placeholders::_1,\n'
        '        std::placeholders::_2));\n\n'
        '  initialize_storage();\n',
        "recovery service construction",
    )

    recovery_impl = textwrap.dedent(
        r'''

        void LocationRegistryNode::handle_recover_storage(
          const std::shared_ptr<RecoveryService::Request> request,
          std::shared_ptr<RecoveryService::Response> response)
        {
          std::lock_guard<std::mutex> mutation_lock{
            mutation_mutex_};

          response->recovered = false;
          response->schema_version = 0U;
          response->location_count = 0U;
          response->candidate_count = 0U;
          response->event_count = 0U;
          response->last_event_sequence = 0U;

          if (request->actor_id.empty()) {
            finish_mutation_rejected(
              "storage recovery requires actor_id");

            response->result_code =
              RecoveryService::Response::RESULT_INVALID_REQUEST;
            response->reason =
              "storage recovery requires actor_id";
            publish_status();
            return;
          }

          if (!enable_write_services_) {
            finish_mutation_rejected(
              "write services are disabled");

            response->result_code =
              RecoveryService::Response::RESULT_NOT_ENABLED;
            response->reason =
              "write services are disabled";
            publish_status();
            return;
          }

          {
            std::unique_lock<std::shared_mutex> lock{
              state_mutex_};

            mutation_in_progress_ = true;
            last_mutation_result_ =
              "storage_recovery:in_progress";
          }

          try {
            auto recovered_store =
              std::make_unique<SqliteStore>(database_path_);

            const auto open_result =
              recovered_store->open();

            if (!open_result.success) {
              finish_mutation_degraded(
                "storage recovery open failed: " +
                open_result.reason);

              response->result_code =
                RecoveryService::Response::
                  RESULT_STORAGE_UNAVAILABLE;
              response->reason = open_result.reason;
              publish_status();
              return;
            }

            if (auto_migrate_) {
              SchemaStatus schema_status;

              const auto migration_result =
                recovered_store->migrate(&schema_status);

              if (!migration_result.success) {
                finish_mutation_degraded(
                  "storage recovery migration failed: " +
                  migration_result.reason);

                response->result_code =
                  RecoveryService::Response::
                    RESULT_STORAGE_UNAVAILABLE;
                response->reason = migration_result.reason;
                publish_status();
                return;
              }
            } else {
              std::uint32_t schema_version = 0U;

              const auto version_result =
                recovered_store->schema_version(
                  &schema_version);

              if (
                !version_result.success ||
                schema_version !=
                  kSupportedSqliteSchemaVersion)
              {
                const std::string reason =
                  version_result.success ?
                  "database schema is not current" :
                  version_result.reason;

                finish_mutation_degraded(
                  "storage recovery schema failed: " + reason);

                response->result_code =
                  RecoveryService::Response::
                    RESULT_STORAGE_UNAVAILABLE;
                response->reason = reason;
                publish_status();
                return;
              }
            }

            IntegrityReport integrity;

            const auto integrity_result =
              recovered_store->integrity_check(&integrity);

            if (
              !integrity_result.success ||
              !integrity.healthy)
            {
              const std::string reason =
                integrity_result.reason.empty() ?
                "database integrity validation failed" :
                integrity_result.reason;

              finish_mutation_degraded(
                "storage recovery integrity failed: " + reason);

              response->result_code =
                RecoveryService::Response::
                  RESULT_INTEGRITY_FAILED;
              response->reason = reason;
              publish_status();
              return;
            }

            auto recovered_repository =
              std::make_unique<SqliteRepository>(
                *recovered_store);

            CatalogSnapshot recovered_snapshot;
            BootstrapReport recovered_report;

            const auto bootstrap_result =
              recovered_repository->bootstrap(
                &recovered_snapshot,
                &recovered_report);

            if (!bootstrap_result.success) {
              finish_mutation_degraded(
                "storage recovery bootstrap failed: " +
                bootstrap_result.reason);

              response->result_code =
                RecoveryService::Response::
                  RESULT_STORAGE_UNAVAILABLE;
              response->reason = bootstrap_result.reason;
              publish_status();
              return;
            }

            InMemoryLocationCatalog recovered_catalog;
            std::string hydration_reason;

            if (
              !hydrate_catalog(
                recovered_snapshot,
                &recovered_catalog,
                &hydration_reason))
            {
              finish_mutation_degraded(
                "storage recovery hydration failed: " +
                hydration_reason);

              response->result_code =
                RecoveryService::Response::
                  RESULT_INTEGRITY_FAILED;
              response->reason = hydration_reason;
              publish_status();
              return;
            }

            std::unique_ptr<SqliteStore>
              previous_store;
            std::unique_ptr<SqliteRepository>
              previous_repository;

            {
              std::unique_lock<std::shared_mutex> lock{
                state_mutex_};

              previous_repository =
                std::move(repository_);
              previous_store = std::move(store_);

              store_ = std::move(recovered_store);
              repository_ =
                std::move(recovered_repository);

              catalog_snapshot_ =
                std::move(recovered_snapshot);
              catalog_view_.replace(catalog_snapshot_);
              bootstrap_report_ = recovered_report;

              ready_ = true;
              write_ready_ = true;
              storage_healthy_ = true;
              mutation_in_progress_ = false;
              state_ = "ready";
              reason_ =
                "persistent read/write registry recovered";
              last_mutation_event_sequence_ =
                recovered_report.last_event_sequence;
              last_mutation_result_ =
                "storage_recovered:" + request->actor_id;
            }

            response->recovered = true;
            response->result_code =
              RecoveryService::Response::RESULT_RECOVERED;
            response->reason =
              "storage recovered after verified reload";
            response->schema_version =
              recovered_report.schema_version;
            response->location_count =
              static_cast<std::uint64_t>(
                recovered_report.location_count);
            response->candidate_count =
              static_cast<std::uint64_t>(
                recovered_report.candidate_count);
            response->event_count =
              recovered_report.event_count;
            response->last_event_sequence =
              recovered_report.last_event_sequence;

            if (publish_snapshot_enabled_) {
              publish_snapshot();
            }

            publish_status();
          }
          catch (const std::exception & exception) {
            finish_mutation_degraded(
              "storage recovery exception: " +
              std::string(exception.what()));

            response->result_code =
              RecoveryService::Response::RESULT_INTERNAL_ERROR;
            response->reason = exception.what();
            publish_status();
          }
        }
        ''')

    source = replace_once(
        source,
        "\n}  // namespace savo_locations\n",
        recovery_impl + "\n}  // namespace savo_locations\n",
        "recovery callback implementation",
    )
    write(source_path, source)

    config_path = locations / "config" / "locations_node.yaml"
    config = config_path.read_text(encoding="utf-8")
    config += textwrap.dedent(
        """

        # Fail-closed recovery is explicit. When a write transaction reports a
        # storage failure, write_ready remains false until an operator calls:
        #   /savo_locations/storage/recover
        # Recovery opens and validates a fresh SQLite connection before the
        # active repository is replaced. Read services keep the last committed
        # in-memory catalog while write readiness is degraded.
        """
    )
    write(config_path, config)

    # New runtime hardening test.
    hardening_test = textwrap.dedent(
        r'''
        // Copyright 2026 Ahnaf Tahmid
        // SPDX-License-Identifier: LicenseRef-Proprietary

        #include <sqlite3.h>

        #include <chrono>
        #include <filesystem>
        #include <future>
        #include <memory>
        #include <string>
        #include <thread>

        #include "gtest/gtest.h"
        #include "rclcpp/rclcpp.hpp"

        #include "savo_msgs/srv/approve_location.hpp"
        #include "savo_msgs/srv/list_locations.hpp"
        #include "savo_msgs/srv/recover_location_storage.hpp"
        #include "savo_msgs/srv/register_location_candidate.hpp"

        #include "savo_locations/location_registry_node.hpp"
        #include "savo_locations/sqlite_repository.hpp"
        #include "savo_locations/sqlite_store.hpp"

        namespace
        {

        using namespace std::chrono_literals;

        class RclcppGuard
        {
        public:
          RclcppGuard()
          {
            int argc = 0;
            char ** argv = nullptr;
            rclcpp::init(argc, argv);
          }

          ~RclcppGuard()
          {
            if (rclcpp::ok()) {
              rclcpp::shutdown();
            }
          }
        };

        class ExecutorThread
        {
        public:
          explicit ExecutorThread(
            rclcpp::executors::MultiThreadedExecutor & executor)
          : executor_(executor),
            thread_([this]() {executor_.spin();})
          {
          }

          ~ExecutorThread()
          {
            executor_.cancel();
            if (thread_.joinable()) {
              thread_.join();
            }
          }

        private:
          rclcpp::executors::MultiThreadedExecutor & executor_;
          std::thread thread_;
        };

        std::filesystem::path database_path(
          const std::string & name)
        {
          const std::filesystem::path directory{
            SAVO_LOCATIONS_TEST_DB_DIR};

          std::filesystem::create_directories(directory);

          const auto path = directory / name;

          std::filesystem::remove(path);
          std::filesystem::remove(path.string() + "-wal");
          std::filesystem::remove(path.string() + "-shm");

          return path;
        }

        rclcpp::NodeOptions node_options(
          const std::filesystem::path & path)
        {
          rclcpp::NodeOptions options;
          options.parameter_overrides({
              rclcpp::Parameter("database_path", path.string()),
              rclcpp::Parameter("create_parent_directories", true),
              rclcpp::Parameter("auto_migrate", true),
              rclcpp::Parameter("enable_write_services", true),
              rclcpp::Parameter("status_publish_hz", 20.0),
              rclcpp::Parameter("heartbeat_publish_hz", 20.0),
              rclcpp::Parameter("publish_snapshot", true),
            });
          return options;
        }

        savo_msgs::msg::LocationCandidate make_candidate()
        {
          savo_msgs::msg::LocationCandidate candidate;
          candidate.candidate_id = "candidate-hardening-27";
          candidate.map_id = "campus_main";
          candidate.map_revision = 7U;
          candidate.map_release_id = "campus_main_release_2026_07";
          candidate.tag_family = "tag36h11";
          candidate.tag_id = 27;
          candidate.tag_pose_map.header.frame_id = "map";
          candidate.tag_pose_map.pose.position.x = 12.8;
          candidate.tag_pose_map.pose.position.y = 8.1;
          candidate.tag_pose_map.pose.orientation.w = 1.0;
          candidate.detection_quality = 0.95F;
          candidate.accepted_observations = 8U;
          candidate.position_stddev_m = 0.02F;
          candidate.yaw_stddev_rad = 0.03F;
          candidate.approach_pose_valid = true;
          candidate.approach_pose.header.frame_id = "map";
          candidate.approach_pose.pose.position.x = 12.0;
          candidate.approach_pose.pose.position.y = 7.5;
          candidate.approach_pose.pose.orientation.w = 1.0;
          candidate.suggested_location_id = "A201";
          candidate.suggested_display_name = "Room A201";
          candidate.suggested_aliases = {"East classroom"};
          candidate.suggested_semantic_type = "classroom";
          candidate.source_session_id = "mapping-session-7";
          candidate.source_component = "savo_mapping";
          return candidate;
        }

        template<typename FutureT>
        bool future_ready(FutureT & future)
        {
          return future.wait_for(3s) == std::future_status::ready;
        }

        void execute_sql(
          const std::filesystem::path & path,
          const char * sql)
        {
          sqlite3 * database = nullptr;
          ASSERT_EQ(
            sqlite3_open_v2(
              path.c_str(),
              &database,
              SQLITE_OPEN_READWRITE | SQLITE_OPEN_FULLMUTEX,
              nullptr),
            SQLITE_OK);

          char * error = nullptr;
          const int code = sqlite3_exec(
            database, sql, nullptr, nullptr, &error);
          const std::string message =
            error == nullptr ? "" : error;
          sqlite3_free(error);
          sqlite3_close(database);
          ASSERT_EQ(code, SQLITE_OK) << message;
        }

        void install_event_failure_trigger(
          const std::filesystem::path & path)
        {
          execute_sql(
            path,
            "CREATE TRIGGER force_location_event_failure "
            "BEFORE INSERT ON location_events "
            "BEGIN "
            "SELECT RAISE(ABORT, 'forced event failure'); "
            "END;");
        }

        void remove_event_failure_trigger(
          const std::filesystem::path & path)
        {
          execute_sql(
            path,
            "DROP TRIGGER force_location_event_failure;");
        }

        std::shared_ptr<
          savo_msgs::srv::RegisterLocationCandidate::Response>
        register_candidate(
          const std::shared_ptr<rclcpp::Node> & client_node,
          const std::string & actor)
        {
          auto client = client_node->create_client<
            savo_msgs::srv::RegisterLocationCandidate>(
              "/savo_locations/candidates/register");
          EXPECT_TRUE(client->wait_for_service(3s));

          auto request = std::make_shared<
            savo_msgs::srv::RegisterLocationCandidate::Request>();
          request->candidate = make_candidate();
          request->actor_id = actor;

          auto future = client->async_send_request(request);
          EXPECT_TRUE(future_ready(future));
          return future.get();
        }

        }  // namespace

        TEST(RegistryHardeningNode, ConcurrentApprovalsCommitExactlyOnce)
        {
          RclcppGuard guard;
          const auto path = database_path(
            "loc3p_concurrent_approval.sqlite3");

          auto registry = std::make_shared<
            savo_locations::LocationRegistryNode>(
              node_options(path));
          auto client_node = std::make_shared<rclcpp::Node>(
            "savo_locations_concurrent_client");

          rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 4U);
          executor.add_node(registry);
          executor.add_node(client_node);
          ExecutorThread spinning(executor);

          const auto registered = register_candidate(
            client_node, "mapping_operator");
          ASSERT_TRUE(registered->registered);

          auto approve_client = client_node->create_client<
            savo_msgs::srv::ApproveLocation>(
              "/savo_locations/candidates/approve");
          ASSERT_TRUE(approve_client->wait_for_service(3s));

          auto request_a = std::make_shared<
            savo_msgs::srv::ApproveLocation::Request>();
          request_a->candidate_id = "candidate-hardening-27";
          request_a->expected_candidate_revision = 1U;
          request_a->actor_id = "operator_a";
          request_a->arrival_confirmation_required = true;

          auto request_b = std::make_shared<
            savo_msgs::srv::ApproveLocation::Request>(*request_a);
          request_b->actor_id = "operator_b";

          auto future_a = approve_client->async_send_request(request_a);
          auto future_b = approve_client->async_send_request(request_b);

          ASSERT_TRUE(future_ready(future_a));
          ASSERT_TRUE(future_ready(future_b));

          const auto response_a = future_a.get();
          const auto response_b = future_b.get();

          const int success_count =
            static_cast<int>(response_a->approved) +
            static_cast<int>(response_b->approved);

          EXPECT_EQ(success_count, 1);
          EXPECT_TRUE(registry->registry_ready());
          EXPECT_TRUE(registry->registry_write_ready());

          savo_locations::SqliteStore store{path.string()};
          ASSERT_TRUE(store.open().success);
          savo_locations::SqliteRepository repository{store};
          savo_locations::CatalogSnapshot snapshot;
          savo_locations::BootstrapReport report;
          ASSERT_TRUE(repository.bootstrap(&snapshot, &report).success);
          EXPECT_EQ(snapshot.locations.size(), 1U);
          EXPECT_EQ(snapshot.candidates.size(), 1U);
          EXPECT_EQ(report.event_count, 2U);
        }

        TEST(RegistryHardeningNode, VerifiedRecoveryRestoresWritesAfterRollback)
        {
          RclcppGuard guard;
          const auto path = database_path(
            "loc3p_verified_recovery.sqlite3");

          auto registry = std::make_shared<
            savo_locations::LocationRegistryNode>(
              node_options(path));
          auto client_node = std::make_shared<rclcpp::Node>(
            "savo_locations_recovery_client");

          rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 4U);
          executor.add_node(registry);
          executor.add_node(client_node);
          ExecutorThread spinning(executor);

          const auto registered = register_candidate(
            client_node, "mapping_operator");
          ASSERT_TRUE(registered->registered);
          ASSERT_EQ(registered->stored_candidate.candidate_revision, 1U);

          install_event_failure_trigger(path);

          auto approve_client = client_node->create_client<
            savo_msgs::srv::ApproveLocation>(
              "/savo_locations/candidates/approve");
          ASSERT_TRUE(approve_client->wait_for_service(3s));

          auto approve_request = std::make_shared<
            savo_msgs::srv::ApproveLocation::Request>();
          approve_request->candidate_id = "candidate-hardening-27";
          approve_request->expected_candidate_revision = 1U;
          approve_request->actor_id = "location_operator";
          approve_request->arrival_confirmation_required = true;

          auto failed_future =
            approve_client->async_send_request(approve_request);
          ASSERT_TRUE(future_ready(failed_future));
          const auto failed_response = failed_future.get();

          EXPECT_FALSE(failed_response->approved);
          EXPECT_EQ(
            failed_response->result_code,
            savo_msgs::srv::ApproveLocation::Response::
              RESULT_STORAGE_UNAVAILABLE);
          EXPECT_TRUE(registry->registry_ready());
          EXPECT_FALSE(registry->registry_write_ready());

          auto list_client = client_node->create_client<
            savo_msgs::srv::ListLocations>(
              "/savo_locations/list");
          ASSERT_TRUE(list_client->wait_for_service(3s));
          auto list_future = list_client->async_send_request(
            std::make_shared<
              savo_msgs::srv::ListLocations::Request>());
          ASSERT_TRUE(future_ready(list_future));
          EXPECT_TRUE(list_future.get()->locations.empty());

          remove_event_failure_trigger(path);

          auto recovery_client = client_node->create_client<
            savo_msgs::srv::RecoverLocationStorage>(
              "/savo_locations/storage/recover");
          ASSERT_TRUE(recovery_client->wait_for_service(3s));

          auto recovery_request = std::make_shared<
            savo_msgs::srv::RecoverLocationStorage::Request>();
          recovery_request->actor_id = "maintenance_operator";
          recovery_request->reason = "event trigger removed";

          auto recovery_future =
            recovery_client->async_send_request(recovery_request);
          ASSERT_TRUE(future_ready(recovery_future));
          const auto recovery_response = recovery_future.get();

          ASSERT_TRUE(recovery_response->recovered);
          EXPECT_EQ(recovery_response->candidate_count, 1U);
          EXPECT_EQ(recovery_response->location_count, 0U);
          EXPECT_EQ(recovery_response->event_count, 1U);
          EXPECT_EQ(recovery_response->last_event_sequence, 1U);
          EXPECT_TRUE(registry->registry_write_ready());

          auto retry_future =
            approve_client->async_send_request(approve_request);
          ASSERT_TRUE(future_ready(retry_future));
          const auto retry_response = retry_future.get();

          ASSERT_TRUE(retry_response->approved);
          EXPECT_EQ(retry_response->location.location_id, "A201");
          EXPECT_EQ(retry_response->location.record_revision, 1U);

          savo_locations::SqliteStore store{path.string()};
          ASSERT_TRUE(store.open().success);
          savo_locations::SqliteRepository repository{store};
          savo_locations::CatalogSnapshot snapshot;
          savo_locations::BootstrapReport report;
          ASSERT_TRUE(repository.bootstrap(&snapshot, &report).success);
          EXPECT_EQ(snapshot.locations.size(), 1U);
          EXPECT_EQ(report.event_count, 2U);
          EXPECT_EQ(report.last_event_sequence, 2U);
        }
        ''')
    hardening_test_path = locations / "test" / "ros" / "test_registry_hardening_node.cpp"
    write(hardening_test_path, hardening_test)

    phase_contract = textwrap.dedent(
        """\
        from pathlib import Path
        import xml.etree.ElementTree as ET


        ROOT = Path(__file__).resolve().parents[2]


        def read(relative: str) -> str:
            return (ROOT / relative).read_text(encoding="utf-8")


        def parse_version(value: str) -> tuple[int, ...]:
            return tuple(int(part) for part in value.split("."))


        def test_package_contains_loc3p_hardening_or_later() -> None:
            package = ET.parse(ROOT / "package.xml").getroot()
            version = package.findtext("version")
            assert version is not None
            assert parse_version(version) >= (0, 11, 0)
            assert f'"{version}"' in read(
                "include/savo_locations/constants.hpp"
            )


        def test_verified_storage_recovery_is_wired() -> None:
            header = read(
                "include/savo_locations/location_registry_node.hpp"
            )
            source = read("src/location_registry_node.cpp")
            names = read(
                "include/savo_locations/service_names.hpp"
            )

            for token in (
                "RecoverLocationStorage",
                "handle_recover_storage",
                "recovery_service_",
            ):
                assert token in header or token in source

            assert "kRecoverStorage" in names
            assert "/savo_locations/storage/recover" in names
            assert "integrity_check" in source
            assert "bootstrap" in source
            assert "hydrate_catalog" in source
            assert "persistent read/write registry recovered" in source


        def test_failed_writes_remain_fail_closed() -> None:
            source = read("src/location_registry_node.cpp")

            assert 'state_ = "degraded_write"' in source
            assert "write_ready_ = false" in source
            degraded_body = source[
                source.index("finish_mutation_degraded"):
                source.index("finish_mutation_committed")
            ]
            assert "\\n  ready_ = false;" not in degraded_body


        def test_runtime_hardening_tests_are_registered() -> None:
            cmake = read("CMakeLists.txt")
            test_source = read(
                "test/ros/test_registry_hardening_node.cpp"
            )

            assert "test_registry_hardening_node" in cmake
            assert "test_phase3p_hardening_contracts" in cmake

            for token in (
                "ConcurrentApprovalsCommitExactlyOnce",
                "VerifiedRecoveryRestoresWritesAfterRollback",
                "force_location_event_failure",
                "RESULT_STORAGE_UNAVAILABLE",
                "last_event_sequence",
            ):
                assert token in test_source
        """
    )
    phase_contract_path = locations / "test" / "contracts" / "test_phase3p_hardening_contracts.py"
    write(phase_contract_path, phase_contract)

    locations_cmake = locations_cmake_path.read_text(encoding="utf-8")
    insertion = textwrap.dedent(
        """

        ament_add_gtest(
          test_registry_hardening_node
          test/ros/test_registry_hardening_node.cpp
        )

        if(TARGET test_registry_hardening_node)
          target_link_libraries(
            test_registry_hardening_node
            savo_locations_ros
            SQLite::SQLite3
          )

          ament_target_dependencies(
            test_registry_hardening_node
            rclcpp
            std_msgs
            savo_msgs
          )

          target_compile_definitions(
            test_registry_hardening_node
            PRIVATE
              "SAVO_LOCATIONS_TEST_DB_DIR=\\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\\""
          )
        endif()

        ament_add_pytest_test(
          test_phase3p_hardening_contracts
          test/contracts/test_phase3p_hardening_contracts.py
          TIMEOUT 60
        )
        """
    )
    locations_cmake = replace_once(
        locations_cmake,
        "\nendif()\n\n# ---------------------------------------------------------------------------\n# Exports\n",
        insertion + "\nendif()\n\n# ---------------------------------------------------------------------------\n# Exports\n",
        "hardening CTest registration",
    )
    write(locations_cmake_path, locations_cmake)

    readme_path = locations / "README.md"
    readme = readme_path.read_text(encoding="utf-8")
    readme += textwrap.dedent(
        """

        ## LOC-3P persistent runtime hardening

        A failed SQLite mutation disables further writes while preserving the
        last committed in-memory catalog for read services. Write readiness is
        restored only through `/savo_locations/storage/recover`. Recovery opens
        a new connection, validates schema and SQLite integrity, bootstraps the
        full catalog and event journal into temporary state, validates catalog
        hydration, and swaps the active repository only after all checks pass.

        Simultaneous mutation requests remain serialized. Concurrency tests
        require exactly one commit when two operators attempt to approve the
        same candidate revision.
        """
    )
    write(readme_path, readme)

    # ------------------------------------------------------------------
    # Static verification.
    # ------------------------------------------------------------------
    ET.parse(msgs / "package.xml")
    ET.parse(locations / "package.xml")
    py_compile.compile(str(apriltag_test_path), doraise=True)
    py_compile.compile(str(test_interfaces_path), doraise=True)
    py_compile.compile(str(phase_contract_path), doraise=True)

    required_tokens = {
        msgs / "CMakeLists.txt": ["RecoverLocationStorage.srv"],
        source_path: [
            "handle_recover_storage",
            "integrity_check",
            "storage recovered after verified reload",
        ],
        locations_cmake_path: [
            "test_registry_hardening_node",
            "test_phase3p_hardening_contracts",
        ],
    }
    for path, tokens in required_tokens.items():
        text = path.read_text(encoding="utf-8")
        for token in tokens:
            if token not in text:
                raise RuntimeError(f"verification failed: {token} missing from {path}")

    modified = [
        msgs / "package.xml",
        msgs / "CMakeLists.txt",
        msgs / "srv" / "RecoverLocationStorage.srv",
        apriltag_test_path,
        test_interfaces_path,
        locations / "package.xml",
        locations_cmake_path,
        constants_path,
        service_names_path,
        header_path,
        source_path,
        config_path,
        hardening_test_path,
        phase_contract_path,
        readme_path,
    ]

    manifest_lines = [
        f"{sha256(backup)}  {backup}",
        "",
        "# Modified files after LOC-3P-B installation",
    ]
    for path in modified:
        manifest_lines.append(f"{sha256(path)}  {path}")
    manifest.write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")

    print("LOC-3P-B persistent runtime hardening installed and verified.")
    print(f"Backup : {backup}")
    print(f"Manifest: {manifest}")
    print("savo_msgs version: 0.4.0")
    print("savo_locations version: 0.11.0")
    print("Expected savo_locations CTest targets after build: 24")
    print("Recovery service: /savo_locations/storage/recover")
    print("Verified reload before write recovery: enabled")
    print("Concurrent approval and rollback/recovery tests: added")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
