#!/usr/bin/env python3
from __future__ import annotations

import ast
from datetime import datetime
import hashlib
from pathlib import Path
import tarfile


ROOT = Path.home() / "Savo_Pi"

PACKAGE = (
    ROOT
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

BACKUPS = ROOT / "backups"
LOGS = ROOT / "change_logs"

EXPECTED_HASHES = {'CMakeLists.txt': 'ac691210c2e2e7e440d51d15750e5507ac77a355b571a7eb1c80754415c93ba6',
 'include/savo_locations/constants.hpp': 'ed903a2117fae60a404c9e943aa16034fd130722f9446ca9cd93976a1f03ecc2',
 'include/savo_locations/sqlite_repository.hpp': '2e246767405b3848b1bfc92b16c2cf387c774a88289d57675e1f9d41f2a85788',
 'package.xml': '1152f1bfbb6ea3fba3e01f0701ddba8c9715d62cecde22c08246a26ca24ccc7c',
 'src/sqlite_repository.cpp': 'caec8c8f8524eb63aaf690aa1849c1a01b51fd20992c70d8e30ba145fa77b0c8'}

FINAL_HASHES = {'CMakeLists.txt': 'c2b6c4ba930b07a35a48b846c0d7a212bed4b5a70f3beb72a9687f4daa2fbfda',
 'include/savo_locations/constants.hpp': 'c2b7932306927699fd5cca20b7832da374696a158cb094b3d1a94abaabc69d3a',
 'include/savo_locations/sqlite_repository.hpp': 'c16dcb0a1c41c6507fefdfd63afc030ab3cef0ccc487f6d26a6a2b42db154c4c',
 'package.xml': '928d632bebf8c7da04b69646d9bf5bd978b65d4d3198d0fb425af135fd9e8e19',
 'src/sqlite_repository.cpp': '8ecfcd5ed697ba994999a7adbd3b21ab924c6b0bc638bc632f95a01cd8cffc51',
 'test/contracts/test_phase3b1_contracts.py': '629269c82394faf5c0403ba36f71d38bab5fa1b0ea620688cf7479470ad10a46',
 'test/storage/test_persistent_mutation_commits.cpp': '2806a898bd173a87f910b9d0f1e88b29c53188f8dea34bbf13520b65cda59626'}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(
            lambda: stream.read(1024 * 1024),
            b"",
        ):
            digest.update(block)

    return digest.hexdigest()


def replace_once(
    text: str,
    old: str,
    new: str,
    label: str,
) -> str:
    count = text.count(old)

    if count != 1:
        raise RuntimeError(
            f"{label}: expected one anchor, found {count}"
        )

    return text.replace(old, new, 1)


def already_applied() -> bool:
    for relative, expected_hash in FINAL_HASHES.items():
        path = PACKAGE / relative

        if (
            not path.is_file() or
            sha256(path) != expected_hash
        ):
            return False

    return True


def verify_input() -> None:
    for relative, expected_hash in EXPECTED_HASHES.items():
        path = PACKAGE / relative

        if not path.is_file():
            raise RuntimeError(
                f"Required file missing: {path}"
            )

        actual = sha256(path)

        if actual != expected_hash:
            raise RuntimeError(
                f"Unexpected LOC-3A source for {relative}\n"
                f"expected: {expected_hash}\n"
                f"actual  : {actual}"
            )

    for relative in (
        "test/storage/"
        "test_persistent_mutation_commits.cpp",
        "test/contracts/"
        "test_phase3b1_contracts.py",
    ):
        path = PACKAGE / relative

        if path.exists():
            raise RuntimeError(
                f"Unexpected pre-existing LOC-3B1 file: {path}"
            )


def patch_header() -> None:
    path = (
        PACKAGE
        / "include"
        / "savo_locations"
        / "sqlite_repository.hpp"
    )

    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        '  kStaleRevision,\n  kApprovalDeltaInvalid,\n  kEventJournalError,\n};',
        '  kStaleRevision,\n  kCandidateRegistrationDeltaInvalid,\n  kApprovalDeltaInvalid,\n  kLocationEnabledDeltaInvalid,\n  kEventJournalError,\n};',
        "SnapshotCode extension",
    )

    text = replace_once(
        text,
        'struct CandidateApprovalCommit\n{\n  std::string candidate_id;\n',
        'struct CandidateRegistrationCommit\n{\n  std::string candidate_id;\n\n  std::string actor_id;\n  std::string reason;\n  std::string payload_json{"{}"};\n\n  CatalogSnapshot post_registration_snapshot;\n};\n\n\n' + 'struct CandidateApprovalCommit\n{\n  std::string candidate_id;\n',
        "CandidateRegistrationCommit insertion",
    )

    text = replace_once(
        text,
        '  CatalogSnapshot post_approval_snapshot;\n};\n\n\nclass SqliteRepository\n',
        '  CatalogSnapshot post_approval_snapshot;\n};\n\n\nstruct LocationEnabledCommit\n{\n  std::string location_id;\n\n  std::uint64_t\n    expected_record_revision{0U};\n\n  bool enabled{false};\n\n  std::string actor_id;\n  std::string reason;\n  std::string payload_json{"{}"};\n\n  CatalogSnapshot post_update_snapshot;\n};\n\n\nclass SqliteRepository\n',
        "LocationEnabledCommit insertion",
    )

    text = replace_once(
        text,
        '[[nodiscard]]\nSnapshotResult commit_candidate_approval(\n  const CandidateApprovalCommit & request,\n  std::uint64_t * event_sequence);\n\nprivate:\n',
        '[[nodiscard]]\nSnapshotResult commit_candidate_registration(\n  const CandidateRegistrationCommit & request,\n  std::uint64_t * event_sequence);\n\n[[nodiscard]]\nSnapshotResult commit_candidate_approval(\n  const CandidateApprovalCommit & request,\n  std::uint64_t * event_sequence);\n\n[[nodiscard]]\nSnapshotResult commit_location_enabled(\n  const LocationEnabledCommit & request,\n  std::uint64_t * event_sequence);\n\nprivate:\n',
        "repository method declarations",
    )

    path.write_text(text, encoding="utf-8")


def patch_repository() -> None:
    path = PACKAGE / "src" / "sqlite_repository.cpp"

    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        'SnapshotResult validate_approval_delta(\n  const CatalogSnapshot & current,\n  const CandidateApprovalCommit & request)\n',
        'SnapshotResult validate_registration_delta(\n  const CatalogSnapshot & current,\n  const CandidateRegistrationCommit & request)\n{\n  const auto & post =\n    request.post_registration_snapshot;\n\n  if (\n    find_candidate(\n      current,\n      request.candidate_id) != nullptr)\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kCandidateRegistrationDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "candidate registration ID already exists");\n  }\n\n  if (\n    post.locations.size() !=\n    current.locations.size())\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kCandidateRegistrationDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "candidate registration changed locations");\n  }\n\n  for (\n    const auto & previous :\n    current.locations)\n  {\n    const auto * next =\n      find_location(\n        post,\n        previous.location.location_id);\n\n    if (\n      next == nullptr ||\n      !same_location_record(\n        previous,\n        *next))\n    {\n      return snapshot_failure(\n        SnapshotCode::\n          kCandidateRegistrationDeltaInvalid,\n        SQLITE_CONSTRAINT,\n        "candidate registration changed "\n        "an existing location");\n    }\n  }\n\n  if (\n    post.candidates.size() !=\n    current.candidates.size() + 1U)\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kCandidateRegistrationDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "candidate registration must add "\n      "exactly one candidate");\n  }\n\n  for (\n    const auto & previous :\n    current.candidates)\n  {\n    const auto * next =\n      find_candidate(\n        post,\n        previous.candidate.candidate_id);\n\n    if (\n      next == nullptr ||\n      !same_candidate_record(\n        previous,\n        *next))\n    {\n      return snapshot_failure(\n        SnapshotCode::\n          kCandidateRegistrationDeltaInvalid,\n        SQLITE_CONSTRAINT,\n        "candidate registration changed "\n        "an existing candidate");\n    }\n  }\n\n  const auto * registered =\n    find_candidate(\n      post,\n      request.candidate_id);\n\n  if (\n    registered == nullptr ||\n    registered->state !=\n      CandidateState::kPendingReview ||\n    registered->candidate_revision != 1U ||\n    registered->candidate.candidate_id !=\n      request.candidate_id ||\n    !registered->review_reason.empty() ||\n    !registered->approved_location_id.empty())\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kCandidateRegistrationDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "registered candidate transition is invalid");\n  }\n\n  return snapshot_success(\n    "candidate registration delta is valid");\n}\n\n\nSnapshotResult validate_location_enabled_delta(\n  const CatalogSnapshot & current,\n  const LocationEnabledCommit & request)\n{\n  const auto & post =\n    request.post_update_snapshot;\n\n  const auto * current_location =\n    find_location(\n      current,\n      request.location_id);\n\n  if (current_location == nullptr) {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_NOTFOUND,\n      "location does not exist");\n  }\n\n  if (\n    current_location->record_revision !=\n    request.expected_record_revision)\n  {\n    return snapshot_failure(\n      SnapshotCode::kStaleRevision,\n      SQLITE_BUSY,\n      "location revision is stale");\n  }\n\n  if (\n    current_location->state ==\n      LocationState::kRetired)\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "retired location cannot change enablement");\n  }\n\n  if (\n    current_location->enabled ==\n    request.enabled)\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "location already has the requested "\n      "enablement state");\n  }\n\n  if (\n    request.expected_record_revision ==\n    std::numeric_limits<\n      std::uint64_t>::max())\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "location revision cannot be incremented");\n  }\n\n  if (\n    post.candidates.size() !=\n    current.candidates.size())\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "enablement change modified candidates");\n  }\n\n  for (\n    const auto & previous :\n    current.candidates)\n  {\n    const auto * next =\n      find_candidate(\n        post,\n        previous.candidate.candidate_id);\n\n    if (\n      next == nullptr ||\n      !same_candidate_record(\n        previous,\n        *next))\n    {\n      return snapshot_failure(\n        SnapshotCode::\n          kLocationEnabledDeltaInvalid,\n        SQLITE_CONSTRAINT,\n        "enablement change modified a candidate");\n    }\n  }\n\n  if (\n    post.locations.size() !=\n    current.locations.size())\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "enablement change modified location count");\n  }\n\n  for (\n    const auto & previous :\n    current.locations)\n  {\n    const auto * next =\n      find_location(\n        post,\n        previous.location.location_id);\n\n    if (next == nullptr) {\n      return snapshot_failure(\n        SnapshotCode::\n          kLocationEnabledDeltaInvalid,\n        SQLITE_CONSTRAINT,\n        "enablement change removed a location");\n    }\n\n    if (\n      previous.location.location_id ==\n      request.location_id)\n    {\n      continue;\n    }\n\n    if (\n      !same_location_record(\n        previous,\n        *next))\n    {\n      return snapshot_failure(\n        SnapshotCode::\n          kLocationEnabledDeltaInvalid,\n        SQLITE_CONSTRAINT,\n        "enablement change modified "\n        "an unrelated location");\n    }\n  }\n\n  const auto * updated =\n    find_location(\n      post,\n      request.location_id);\n\n  if (\n    updated == nullptr ||\n    updated->state !=\n      current_location->state ||\n    updated->enabled != request.enabled ||\n    updated->record_revision !=\n      request.expected_record_revision + 1U ||\n    updated->source_candidate_id !=\n      current_location->source_candidate_id ||\n    !same_location_draft(\n      updated->location,\n      current_location->location))\n  {\n    return snapshot_failure(\n      SnapshotCode::\n        kLocationEnabledDeltaInvalid,\n      SQLITE_CONSTRAINT,\n      "location enablement transition is invalid");\n  }\n\n  return snapshot_success(\n    "location enablement delta is valid");\n}\n\n\n' + 'SnapshotResult validate_approval_delta(\n  const CatalogSnapshot & current,\n  const CandidateApprovalCommit & request)\n',
        "mutation delta validators",
    )

    text = replace_once(
        text,
        '    case SnapshotCode::kStaleRevision:\n      return "stale_revision";\n\n    case SnapshotCode::kApprovalDeltaInvalid:\n      return "approval_delta_invalid";\n',
        '    case SnapshotCode::kStaleRevision:\n      return "stale_revision";\n\n    case SnapshotCode::\n        kCandidateRegistrationDeltaInvalid:\n      return "candidate_registration_delta_invalid";\n\n    case SnapshotCode::kApprovalDeltaInvalid:\n      return "approval_delta_invalid";\n\n    case SnapshotCode::\n        kLocationEnabledDeltaInvalid:\n      return "location_enabled_delta_invalid";\n',
        "SnapshotCode strings",
    )

    text = replace_once(
        text,
        'SnapshotResult\nSqliteRepository::commit_candidate_approval(',
        'SnapshotResult\nSqliteRepository::commit_candidate_registration(\n  const CandidateRegistrationCommit & request,\n  std::uint64_t * event_sequence)\n{\n  if (\n    trim_ascii(request.candidate_id).empty() ||\n    trim_ascii(request.actor_id).empty() ||\n    trim_ascii(request.reason).empty())\n  {\n    return snapshot_failure(\n      SnapshotCode::kInvalidArgument,\n      SQLITE_MISUSE,\n      "candidate ID, actor and reason are required");\n  }\n\n  const auto post_validation =\n    validate_snapshot(\n      request.post_registration_snapshot);\n\n  if (!post_validation.success) {\n    return post_validation;\n  }\n\n  std::lock_guard<std::mutex> lock{\n    store_.mutex_};\n\n  if (store_.database_ == nullptr) {\n    return snapshot_failure(\n      SnapshotCode::kStoreNotOpen,\n      SQLITE_MISUSE,\n      "SQLite store is not open");\n  }\n\n  if (store_.transaction_active_) {\n    return snapshot_failure(\n      SnapshotCode::kTransactionActive,\n      SQLITE_BUSY,\n      "SQLite store already has an active transaction");\n  }\n\n  int code = execute_sql(\n    store_.database_,\n    "BEGIN IMMEDIATE;");\n\n  if (code != SQLITE_OK) {\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not begin candidate registration "\n      "transaction");\n  }\n\n  store_.transaction_active_ = true;\n\n  store_.transaction_owner_ =\n    std::this_thread::get_id();\n\n  auto rollback =\n    [&]()\n    {\n      static_cast<void>(\n        execute_sql(\n          store_.database_,\n          "ROLLBACK;"));\n\n      store_.transaction_active_ = false;\n      store_.transaction_owner_ =\n        std::thread::id{};\n    };\n\n  CatalogSnapshot current;\n\n  auto result = read_locations(\n    store_.database_,\n    &current);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  result = read_candidates(\n    store_.database_,\n    &current);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  result = validate_snapshot(current);\n\n  if (!result.success) {\n    rollback();\n\n    result.code =\n      SnapshotCode::kCorruptData;\n\n    result.sqlite_code =\n      SQLITE_CORRUPT;\n\n    result.reason =\n      "persisted pre-registration catalog "\n      "is invalid: " +\n      result.reason;\n\n    return result;\n  }\n\n  result = validate_registration_delta(\n    current,\n    request);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  const std::int64_t timestamp =\n    unix_time_ns();\n\n  code = replace_snapshot_rows(\n    store_.database_,\n    request.post_registration_snapshot,\n    timestamp);\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not persist candidate registration");\n  }\n\n  PersistenceEvent event;\n\n  event.event_time_unix_ns = timestamp;\n\n  event.event_type =\n    PersistenceEventType::\n      kCandidateRegistered;\n\n  event.candidate_id =\n    request.candidate_id;\n\n  event.entity_revision = 1U;\n  event.actor_id = request.actor_id;\n  event.reason = request.reason;\n\n  event.payload_json =\n    request.payload_json.empty() ?\n      "{}" :\n      request.payload_json;\n\n  std::uint64_t inserted_sequence = 0U;\n\n  code = insert_event_row(\n    store_.database_,\n    event,\n    &inserted_sequence);\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return snapshot_failure(\n      SnapshotCode::kEventJournalError,\n      code,\n      "candidate registration event append failed; "\n      "snapshot was rolled back");\n  }\n\n  code = execute_sql(\n    store_.database_,\n    "COMMIT;");\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not commit candidate registration "\n      "transaction");\n  }\n\n  store_.transaction_active_ = false;\n  store_.transaction_owner_ =\n    std::thread::id{};\n\n  if (event_sequence != nullptr) {\n    *event_sequence = inserted_sequence;\n  }\n\n  return snapshot_success(\n    "candidate registration persisted atomically");\n}\n\n\n' +
        'SnapshotResult\nSqliteRepository::commit_candidate_approval(',
        "candidate registration commit",
    )

    namespace_close = (
        "\n}  // namespace savo_locations\n"
    )

    text = replace_once(
        text,
        namespace_close,
        "\n\n" +
        'SnapshotResult\nSqliteRepository::commit_location_enabled(\n  const LocationEnabledCommit & request,\n  std::uint64_t * event_sequence)\n{\n  if (\n    trim_ascii(request.location_id).empty() ||\n    request.expected_record_revision == 0U ||\n    trim_ascii(request.actor_id).empty() ||\n    trim_ascii(request.reason).empty())\n  {\n    return snapshot_failure(\n      SnapshotCode::kInvalidArgument,\n      SQLITE_MISUSE,\n      "location ID, revision, actor and reason "\n      "are required");\n  }\n\n  const auto post_validation =\n    validate_snapshot(\n      request.post_update_snapshot);\n\n  if (!post_validation.success) {\n    return post_validation;\n  }\n\n  std::lock_guard<std::mutex> lock{\n    store_.mutex_};\n\n  if (store_.database_ == nullptr) {\n    return snapshot_failure(\n      SnapshotCode::kStoreNotOpen,\n      SQLITE_MISUSE,\n      "SQLite store is not open");\n  }\n\n  if (store_.transaction_active_) {\n    return snapshot_failure(\n      SnapshotCode::kTransactionActive,\n      SQLITE_BUSY,\n      "SQLite store already has an active transaction");\n  }\n\n  int code = execute_sql(\n    store_.database_,\n    "BEGIN IMMEDIATE;");\n\n  if (code != SQLITE_OK) {\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not begin location enablement "\n      "transaction");\n  }\n\n  store_.transaction_active_ = true;\n\n  store_.transaction_owner_ =\n    std::this_thread::get_id();\n\n  auto rollback =\n    [&]()\n    {\n      static_cast<void>(\n        execute_sql(\n          store_.database_,\n          "ROLLBACK;"));\n\n      store_.transaction_active_ = false;\n      store_.transaction_owner_ =\n        std::thread::id{};\n    };\n\n  CatalogSnapshot current;\n\n  auto result = read_locations(\n    store_.database_,\n    &current);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  result = read_candidates(\n    store_.database_,\n    &current);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  result = validate_snapshot(current);\n\n  if (!result.success) {\n    rollback();\n\n    result.code =\n      SnapshotCode::kCorruptData;\n\n    result.sqlite_code =\n      SQLITE_CORRUPT;\n\n    result.reason =\n      "persisted pre-enablement catalog "\n      "is invalid: " +\n      result.reason;\n\n    return result;\n  }\n\n  result = validate_location_enabled_delta(\n    current,\n    request);\n\n  if (!result.success) {\n    rollback();\n    return result;\n  }\n\n  const std::int64_t timestamp =\n    unix_time_ns();\n\n  code = replace_snapshot_rows(\n    store_.database_,\n    request.post_update_snapshot,\n    timestamp);\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not persist location enablement");\n  }\n\n  PersistenceEvent event;\n\n  event.event_time_unix_ns = timestamp;\n\n  event.event_type =\n    PersistenceEventType::\n      kLocationEnabledChanged;\n\n  event.location_id =\n    request.location_id;\n\n  event.entity_revision =\n    request.expected_record_revision + 1U;\n\n  event.actor_id = request.actor_id;\n  event.reason = request.reason;\n\n  event.payload_json =\n    request.payload_json.empty() ?\n      "{}" :\n      request.payload_json;\n\n  std::uint64_t inserted_sequence = 0U;\n\n  code = insert_event_row(\n    store_.database_,\n    event,\n    &inserted_sequence);\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return snapshot_failure(\n      SnapshotCode::kEventJournalError,\n      code,\n      "location enablement event append failed; "\n      "snapshot was rolled back");\n  }\n\n  code = execute_sql(\n    store_.database_,\n    "COMMIT;");\n\n  if (code != SQLITE_OK) {\n    rollback();\n\n    return sqlite_failure(\n      store_.database_,\n      code,\n      "could not commit location enablement "\n      "transaction");\n  }\n\n  store_.transaction_active_ = false;\n  store_.transaction_owner_ =\n    std::thread::id{};\n\n  if (event_sequence != nullptr) {\n    *event_sequence = inserted_sequence;\n  }\n\n  return snapshot_success(\n    "location enablement persisted atomically");\n}\n' +
        namespace_close,
        "location enablement commit",
    )

    path.write_text(text, encoding="utf-8")



def patch_constants() -> None:
    path = (
        PACKAGE
        / "include"
        / "savo_locations"
        / "constants.hpp"
    )

    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        '"0.8.0"',
        '"0.9.0"',
        "package version constant",
    )

    path.write_text(text, encoding="utf-8")


def patch_cmake() -> None:
    path = PACKAGE / "CMakeLists.txt"
    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "project(savo_locations VERSION 0.8.0 LANGUAGES CXX)",
        "project(savo_locations VERSION 0.9.0 LANGUAGES CXX)",
        "CMake project version",
    )

    text = replace_once(
        text,
        '  ament_add_gtest(\n    test_persistent_catalog\n    test/storage/test_persistent_catalog.cpp\n  )\n\n  if(TARGET test_persistent_catalog)\n    target_link_libraries(\n      test_persistent_catalog\n      savo_locations_storage\n      SQLite::SQLite3\n    )\n\n    target_compile_definitions(\n      test_persistent_catalog\n      PRIVATE\n        "SAVO_LOCATIONS_TEST_DB_DIR=\\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\\""\n    )\n  endif()\n',
        '  ament_add_gtest(\n    test_persistent_catalog\n    test/storage/test_persistent_catalog.cpp\n  )\n\n  if(TARGET test_persistent_catalog)\n    target_link_libraries(\n      test_persistent_catalog\n      savo_locations_storage\n      SQLite::SQLite3\n    )\n\n    target_compile_definitions(\n      test_persistent_catalog\n      PRIVATE\n        "SAVO_LOCATIONS_TEST_DB_DIR=\\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\\""\n    )\n  endif()\n' + "\n" + '  ament_add_gtest(\n    test_persistent_mutation_commits\n    test/storage/test_persistent_mutation_commits.cpp\n  )\n\n  if(TARGET test_persistent_mutation_commits)\n    target_link_libraries(\n      test_persistent_mutation_commits\n      savo_locations_storage\n      SQLite::SQLite3\n    )\n\n    target_compile_definitions(\n      test_persistent_mutation_commits\n      PRIVATE\n        "SAVO_LOCATIONS_TEST_DB_DIR=\\"${CMAKE_CURRENT_BINARY_DIR}/storage_test_runtime\\""\n    )\n  endif()\n',
        "persistent mutation test target",
    )

    text = replace_once(
        text,
        'ament_add_pytest_test(\n  test_phase3a_contracts\n  test/contracts/test_phase3a_contracts.py\n  TIMEOUT 60\n)\n',
        'ament_add_pytest_test(\n  test_phase3a_contracts\n  test/contracts/test_phase3a_contracts.py\n  TIMEOUT 60\n)\n' + "\n" + 'ament_add_pytest_test(\n  test_phase3b1_contracts\n  test/contracts/test_phase3b1_contracts.py\n  TIMEOUT 60\n)\n',
        "LOC-3B1 contract target",
    )

    path.write_text(text, encoding="utf-8")


def patch_package_xml() -> None:
    path = PACKAGE / "package.xml"
    text = path.read_text(encoding="utf-8")

    text = replace_once(
        text,
        "<version>0.8.0</version>",
        "<version>0.9.0</version>",
        "package version",
    )

    path.write_text(text, encoding="utf-8")


def write_tests() -> None:
    storage_test = (
        PACKAGE
        / "test"
        / "storage"
        / "test_persistent_mutation_commits.cpp"
    )

    contract_test = (
        PACKAGE
        / "test"
        / "contracts"
        / "test_phase3b1_contracts.py"
    )

    storage_test.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    contract_test.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    storage_test.write_text(
        '#include <gtest/gtest.h>\n#include <sqlite3.h>\n\n#include <filesystem>\n#include <string>\n\n#include "savo_locations/sqlite_repository.hpp"\n#include "savo_locations/sqlite_schema.hpp"\n#include "savo_locations/sqlite_store.hpp"\n\n\nnamespace\n{\n\nstd::filesystem::path clean_database(\n  const std::string & name)\n{\n  const std::filesystem::path root{\n    SAVO_LOCATIONS_TEST_DB_DIR};\n\n  std::filesystem::create_directories(root);\n\n  const auto path = root / name;\n\n  std::filesystem::remove(path);\n  std::filesystem::remove(\n    path.string() + "-wal");\n\n  std::filesystem::remove(\n    path.string() + "-shm");\n\n  return path;\n}\n\n\nsavo_locations::PoseData pose(\n  const double x,\n  const double y)\n{\n  savo_locations::PoseData value;\n\n  value.frame_id = "map";\n  value.x = x;\n  value.y = y;\n  value.qw = 1.0;\n\n  return value;\n}\n\n\nsavo_locations::CandidateRecordData\npending_candidate()\n{\n  savo_locations::CandidateRecordData record;\n\n  record.state =\n    savo_locations::CandidateState::\n      kPendingReview;\n\n  record.candidate_revision = 1U;\n\n  auto & candidate = record.candidate;\n\n  candidate.candidate_id = "candidate-31";\n  candidate.map.map_id = "campus_main";\n  candidate.map.map_revision = 7U;\n  candidate.map.map_release_id = "release-7";\n  candidate.tag.family = "tag36h11";\n  candidate.tag.id = 31;\n  candidate.tag_pose_map = pose(4.0, 5.0);\n  candidate.detection_quality = 0.95;\n  candidate.accepted_observations = 8U;\n  candidate.position_stddev_m = 0.02;\n  candidate.yaw_stddev_rad = 0.03;\n  candidate.approach_pose = pose(3.5, 4.5);\n  candidate.suggested_location_id = "B301";\n\n  candidate.suggested_display_name =\n    "Room B301";\n\n  candidate.suggested_semantic_type =\n    "classroom";\n\n  candidate.source_session_id = "session-1";\n  candidate.source_component = "savo_mapping";\n\n  return record;\n}\n\n\nsavo_locations::LocationRecordData\nlocation(\n  const bool enabled = true,\n  const std::uint64_t revision = 1U)\n{\n  savo_locations::LocationRecordData record;\n\n  record.state =\n    savo_locations::LocationState::\n      kApproved;\n\n  record.enabled = enabled;\n  record.record_revision = revision;\n\n  auto & value = record.location;\n\n  value.location_id = "B301";\n  value.display_name = "Room B301";\n  value.semantic_type = "classroom";\n  value.map.map_id = "campus_main";\n  value.map.map_revision = 7U;\n  value.map.map_release_id = "release-7";\n  value.approach_pose = pose(3.5, 4.5);\n  value.tag.family = "tag36h11";\n  value.tag.id = 31;\n  value.tag_pose_map = pose(4.0, 5.0);\n\n  record.source_candidate_id =\n    "candidate-31";\n\n  return record;\n}\n\n\nvoid open_and_migrate(\n  savo_locations::SqliteStore * store)\n{\n  ASSERT_TRUE(store->open().success);\n\n  savo_locations::SchemaStatus status;\n\n  ASSERT_TRUE(\n    store->migrate(&status).success);\n\n  EXPECT_EQ(status.current_version, 2U);\n}\n\n\nvoid reject_event_type(\n  const std::filesystem::path & path,\n  const int event_type)\n{\n  sqlite3 * database = nullptr;\n\n  ASSERT_EQ(\n    sqlite3_open_v2(\n      path.string().c_str(),\n      &database,\n      SQLITE_OPEN_READWRITE,\n      nullptr),\n    SQLITE_OK);\n\n  const std::string sql =\n    "CREATE TRIGGER reject_event "\n    "BEFORE INSERT ON location_events "\n    "WHEN NEW.event_type=" +\n    std::to_string(event_type) +\n    " BEGIN "\n    "SELECT RAISE(ABORT,\'event rejected\'); "\n    "END;";\n\n  ASSERT_EQ(\n    sqlite3_exec(\n      database,\n      sql.c_str(),\n      nullptr,\n      nullptr,\n      nullptr),\n    SQLITE_OK);\n\n  ASSERT_EQ(\n    sqlite3_close(database),\n    SQLITE_OK);\n}\n\n\nsavo_locations::CandidateRegistrationCommit\nregistration_request()\n{\n  savo_locations::CandidateRegistrationCommit\n    request;\n\n  request.candidate_id = "candidate-31";\n  request.actor_id = "savo_mapping";\n  request.reason = "candidate confirmed";\n\n  request\n    .post_registration_snapshot\n    .candidates\n    .push_back(\n      pending_candidate());\n\n  return request;\n}\n\n\nsavo_locations::LocationEnabledCommit\ndisable_request(\n  const savo_locations::CatalogSnapshot &\n    current)\n{\n  savo_locations::LocationEnabledCommit request;\n\n  request.location_id = "B301";\n\n  request.expected_record_revision = 1U;\n\n  request.enabled = false;\n  request.actor_id = "operator-ahnaf";\n  request.reason = "maintenance";\n  request.post_update_snapshot = current;\n\n  auto & updated =\n    request\n      .post_update_snapshot\n      .locations\n      .front();\n\n  updated.enabled = false;\n  updated.record_revision = 2U;\n\n  return request;\n}\n\n}  // namespace\n\n\nTEST(\n  PersistentMutationCommits,\n  RegistrationCommitsSnapshotAndEvent)\n{\n  savo_locations::SqliteStore store{\n    ":memory:"};\n\n  open_and_migrate(&store);\n\n  savo_locations::SqliteRepository repository{\n    store};\n\n  std::uint64_t sequence = 0U;\n\n  ASSERT_TRUE(\n    repository\n      .commit_candidate_registration(\n        registration_request(),\n        &sequence)\n      .success);\n\n  EXPECT_EQ(sequence, 1U);\n\n  savo_locations::CatalogSnapshot snapshot;\n  savo_locations::BootstrapReport report;\n\n  ASSERT_TRUE(\n    repository\n      .bootstrap(\n        &snapshot,\n        &report)\n      .success);\n\n  ASSERT_EQ(snapshot.candidates.size(), 1U);\n  EXPECT_EQ(report.event_count, 1U);\n\n  const auto duplicate =\n    repository.commit_candidate_registration(\n      registration_request(),\n      nullptr);\n\n  EXPECT_FALSE(duplicate.success);\n\n  EXPECT_EQ(\n    duplicate.code,\n    savo_locations::SnapshotCode::\n      kCandidateRegistrationDeltaInvalid);\n}\n\n\nTEST(\n  PersistentMutationCommits,\n  RegistrationEventFailureRollsBack)\n{\n  const auto path =\n    clean_database(\n      "registration_event_rollback.sqlite3");\n\n  {\n    savo_locations::SqliteStore store{\n      path.string()};\n\n    open_and_migrate(&store);\n  }\n\n  reject_event_type(path, 3);\n\n  savo_locations::SqliteStore store{\n    path.string()};\n\n  open_and_migrate(&store);\n\n  savo_locations::SqliteRepository repository{\n    store};\n\n  const auto result =\n    repository.commit_candidate_registration(\n      registration_request(),\n      nullptr);\n\n  EXPECT_FALSE(result.success);\n\n  EXPECT_EQ(\n    result.code,\n    savo_locations::SnapshotCode::\n      kEventJournalError);\n\n  savo_locations::CatalogSnapshot snapshot;\n  savo_locations::BootstrapReport report;\n\n  ASSERT_TRUE(\n    repository\n      .bootstrap(\n        &snapshot,\n        &report)\n      .success);\n\n  EXPECT_TRUE(snapshot.candidates.empty());\n  EXPECT_EQ(report.event_count, 0U);\n}\n\n\nTEST(\n  PersistentMutationCommits,\n  EnablementCommitsSnapshotAndEvent)\n{\n  savo_locations::SqliteStore store{\n    ":memory:"};\n\n  open_and_migrate(&store);\n\n  savo_locations::SqliteRepository repository{\n    store};\n\n  savo_locations::CatalogSnapshot current;\n\n  current.locations.push_back(location());\n\n  ASSERT_TRUE(\n    repository\n      .save_snapshot(current)\n      .success);\n\n  std::uint64_t sequence = 0U;\n\n  ASSERT_TRUE(\n    repository\n      .commit_location_enabled(\n        disable_request(current),\n        &sequence)\n      .success);\n\n  EXPECT_EQ(sequence, 1U);\n\n  savo_locations::CatalogSnapshot snapshot;\n  savo_locations::BootstrapReport report;\n\n  ASSERT_TRUE(\n    repository\n      .bootstrap(\n        &snapshot,\n        &report)\n      .success);\n\n  ASSERT_EQ(snapshot.locations.size(), 1U);\n  EXPECT_FALSE(snapshot.locations.front().enabled);\n\n  EXPECT_EQ(\n    snapshot\n      .locations\n      .front()\n      .record_revision,\n    2U);\n\n  EXPECT_EQ(report.event_count, 1U);\n}\n\n\nTEST(\n  PersistentMutationCommits,\n  EnablementRejectsStaleAndNoOp)\n{\n  savo_locations::SqliteStore store{\n    ":memory:"};\n\n  open_and_migrate(&store);\n\n  savo_locations::SqliteRepository repository{\n    store};\n\n  savo_locations::CatalogSnapshot current;\n\n  current.locations.push_back(location());\n\n  ASSERT_TRUE(\n    repository\n      .save_snapshot(current)\n      .success);\n\n  auto stale = disable_request(current);\n\n  stale.expected_record_revision = 2U;\n\n  const auto stale_result =\n    repository.commit_location_enabled(\n      stale,\n      nullptr);\n\n  EXPECT_FALSE(stale_result.success);\n\n  EXPECT_EQ(\n    stale_result.code,\n    savo_locations::SnapshotCode::\n      kStaleRevision);\n\n  savo_locations::LocationEnabledCommit no_op;\n\n  no_op.location_id = "B301";\n\n  no_op.expected_record_revision = 1U;\n\n  no_op.enabled = true;\n  no_op.actor_id = "operator-ahnaf";\n  no_op.reason = "no-op";\n  no_op.post_update_snapshot = current;\n\n  const auto no_op_result =\n    repository.commit_location_enabled(\n      no_op,\n      nullptr);\n\n  EXPECT_FALSE(no_op_result.success);\n\n  EXPECT_EQ(\n    no_op_result.code,\n    savo_locations::SnapshotCode::\n      kLocationEnabledDeltaInvalid);\n}\n\n\nTEST(\n  PersistentMutationCommits,\n  EnablementEventFailureRollsBack)\n{\n  const auto path =\n    clean_database(\n      "enablement_event_rollback.sqlite3");\n\n  {\n    savo_locations::SqliteStore store{\n      path.string()};\n\n    open_and_migrate(&store);\n\n    savo_locations::SqliteRepository repository{\n      store};\n\n    savo_locations::CatalogSnapshot current;\n\n    current.locations.push_back(location());\n\n    ASSERT_TRUE(\n      repository\n        .save_snapshot(current)\n        .success);\n  }\n\n  reject_event_type(path, 5);\n\n  savo_locations::SqliteStore store{\n    path.string()};\n\n  open_and_migrate(&store);\n\n  savo_locations::SqliteRepository repository{\n    store};\n\n  savo_locations::CatalogSnapshot current;\n\n  ASSERT_TRUE(\n    repository\n      .load_snapshot(&current)\n      .success);\n\n  const auto result =\n    repository.commit_location_enabled(\n      disable_request(current),\n      nullptr);\n\n  EXPECT_FALSE(result.success);\n\n  EXPECT_EQ(\n    result.code,\n    savo_locations::SnapshotCode::\n      kEventJournalError);\n\n  savo_locations::CatalogSnapshot snapshot;\n  savo_locations::BootstrapReport report;\n\n  ASSERT_TRUE(\n    repository\n      .bootstrap(\n        &snapshot,\n        &report)\n      .success);\n\n  EXPECT_TRUE(snapshot.locations.front().enabled);\n\n  EXPECT_EQ(\n    snapshot\n      .locations\n      .front()\n      .record_revision,\n    1U);\n\n  EXPECT_EQ(report.event_count, 0U);\n}\n\n\nTEST(\n  PersistentMutationCommits,\n  ResultStringsAreStable)\n{\n  EXPECT_EQ(\n    savo_locations::to_string(\n      savo_locations::SnapshotCode::\n        kCandidateRegistrationDeltaInvalid),\n    "candidate_registration_delta_invalid");\n\n  EXPECT_EQ(\n    savo_locations::to_string(\n      savo_locations::SnapshotCode::\n        kLocationEnabledDeltaInvalid),\n    "location_enabled_delta_invalid");\n}\n',
        encoding="utf-8",
    )

    contract_test.write_text(
        'from pathlib import Path\nimport xml.etree.ElementTree as ET\n\n\nROOT = Path(__file__).resolve().parents[2]\n\n\ndef read(relative: str) -> str:\n    return (ROOT / relative).read_text(\n        encoding="utf-8"\n    )\n\n\ndef parse_version(\n    value: str,\n) -> tuple[int, int, int]:\n    return tuple(\n        int(part)\n        for part in value.split(".")\n    )\n\n\ndef test_package_is_loc3b1_or_later() -> None:\n    package = ET.parse(\n        ROOT / "package.xml"\n    ).getroot()\n\n    version = package.findtext("version")\n\n    assert version is not None\n    assert parse_version(version) >= (0, 9, 0)\n\n\ndef test_atomic_commit_contracts_exist() -> None:\n    header = read(\n        "include/savo_locations/"\n        "sqlite_repository.hpp"\n    )\n\n    for fragment in (\n        "CandidateRegistrationCommit",\n        "LocationEnabledCommit",\n        "commit_candidate_registration",\n        "commit_candidate_approval",\n        "commit_location_enabled",\n        "kCandidateRegistrationDeltaInvalid",\n        "kLocationEnabledDeltaInvalid",\n    ):\n        assert fragment in header\n\n\ndef test_registration_commit_is_transactional() -> None:\n    source = read(\n        "src/sqlite_repository.cpp"\n    )\n\n    for fragment in (\n        "validate_registration_delta",\n        "BEGIN IMMEDIATE;",\n        "kCandidateRegistered",\n        "candidate registration event append failed",\n        "candidate registration persisted atomically",\n    ):\n        assert fragment in source\n\n\ndef test_enablement_commit_is_transactional() -> None:\n    source = read(\n        "src/sqlite_repository.cpp"\n    )\n\n    for fragment in (\n        "validate_location_enabled_delta",\n        "location already has the requested",\n        "kLocationEnabledChanged",\n        "location enablement event append failed",\n        "location enablement persisted atomically",\n    ):\n        assert fragment in source\n\n\ndef test_approval_atomic_commit_is_preserved() -> None:\n    source = read(\n        "src/sqlite_repository.cpp"\n    )\n\n    for fragment in (\n        "validate_approval_delta",\n        "kCandidateApproved",\n        "approval event append failed",\n        "candidate approval persisted atomically",\n    ):\n        assert fragment in source\n\n\ndef test_loc3b1_tests_are_registered() -> None:\n    cmake = read("CMakeLists.txt")\n\n    for target in (\n        "test_persistent_mutation_commits",\n        "test_phase3b1_contracts",\n    ):\n        assert target in cmake\n\n\ndef test_loc3b1_does_not_expose_write_services_yet() -> None:\n    source = read(\n        "src/location_registry_node.cpp"\n    )\n\n    header = read(\n        "include/savo_locations/"\n        "location_registry_node.hpp"\n    )\n\n    combined = source + header\n\n    for forbidden in (\n        "RegisterLocationCandidate",\n        "ApproveLocation",\n        "SetLocationEnabled",\n        "handle_register",\n        "handle_approve",\n        "handle_set_enabled",\n    ):\n        assert forbidden not in combined\n',
        encoding="utf-8",
    )

    ast.parse(
        contract_test.read_text(
            encoding="utf-8"
        )
    )


def verify_output() -> None:
    mismatches = []

    for relative, expected_hash in FINAL_HASHES.items():
        path = PACKAGE / relative

        if not path.is_file():
            mismatches.append(
                f"missing: {relative}"
            )

            continue

        actual = sha256(path)

        if actual != expected_hash:
            mismatches.append(
                f"hash mismatch: {relative}\n"
                f"  expected: {expected_hash}\n"
                f"  actual  : {actual}"
            )

    if mismatches:
        raise RuntimeError(
            "LOC-3B1 output verification failed:\n" +
            "\n".join(mismatches)
        )


def main() -> None:
    if not PACKAGE.is_dir():
        raise SystemExit(
            f"Package directory missing: {PACKAGE}"
        )

    if already_applied():
        print("LOC-3B1 is already applied and verified.")
        return

    verify_input()

    BACKUPS.mkdir(
        parents=True,
        exist_ok=True,
    )

    LOGS.mkdir(
        parents=True,
        exist_ok=True,
    )

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC3B1_savo_locations_{stamp}.tar.gz"
    )

    with tarfile.open(
        backup,
        "w:gz",
    ) as archive:
        archive.add(
            PACKAGE,
            arcname="core/savo_locations",
        )

    patch_header()
    patch_repository()
    patch_constants()
    patch_cmake()
    patch_package_xml()
    write_tests()
    verify_output()

    manifest = (
        LOGS
        / f"LOC3B1_savo_locations_{stamp}.sha256"
    )

    manifest_lines = []

    for relative in sorted(FINAL_HASHES):
        path = PACKAGE / relative

        manifest_lines.append(
            f"{sha256(path)}  "
            f"savo_ws/src/core/savo_locations/"
            f"{relative}"
        )

    manifest.write_text(
        "\n".join(manifest_lines) + "\n",
        encoding="utf-8",
    )

    print("LOC-3B1 installed and verified.")
    print(f"Backup : {backup}")
    print(f"Manifest: {manifest}")
    print("Package version: 0.9.0")
    print("Expected CTest targets after build: 20")
    print(
        "Write ROS services remain intentionally "
        "unexposed until LOC-3B2."
    )


if __name__ == "__main__":
    main()
