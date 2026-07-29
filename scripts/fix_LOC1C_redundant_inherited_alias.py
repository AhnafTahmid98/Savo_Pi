#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import tarfile


ROOT = (
    Path.home()
    / "Savo_Pi"
    / "savo_ws"
    / "src"
    / "core"
    / "savo_locations"
)

SOURCE = ROOT / "src" / "location_catalog.cpp"
TEST = ROOT / "test" / "unit" / "test_location_catalog.cpp"

BACKUPS = Path.home() / "Savo_Pi" / "backups"
LOGS = Path.home() / "Savo_Pi" / "change_logs"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def replace_once(
    text: str,
    old: str,
    new: str,
    label: str,
) -> str:
    count = text.count(old)

    if count == 0:
        if new in text:
            return text

        raise RuntimeError(
            f"Expected block not found: {label}"
        )

    if count != 1:
        raise RuntimeError(
            f"Expected one {label} block, found {count}"
        )

    return text.replace(old, new, 1)


def main() -> None:
    for path in (SOURCE, TEST):
        if not path.is_file():
            raise SystemExit(f"Required file missing: {path}")

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    backup = (
        BACKUPS
        / f"pre_LOC1C_alias_fix_{stamp}.tar.gz"
    )

    with tarfile.open(backup, "w:gz") as archive:
        archive.add(
            SOURCE,
            arcname=(
                "core/savo_locations/"
                "src/location_catalog.cpp"
            ),
        )

        archive.add(
            TEST,
            arcname=(
                "core/savo_locations/"
                "test/unit/test_location_catalog.cpp"
            ),
        )

    source = SOURCE.read_text(encoding="utf-8")

    old_function = '''        std::vector<std::string> choose_aliases(
          const std::vector<std::string> & requested,
          const std::vector<std::string> & suggested)
        {
          const auto & source =
            requested.empty() ?
            suggested :
            requested;

          std::vector<std::string> aliases;
          aliases.reserve(source.size());

          for (const auto & alias : source) {
            aliases.push_back(
              collapse_ascii_whitespace(alias));
          }

          return aliases;
        }
'''

    new_function = '''        std::vector<std::string> choose_aliases(
          const std::vector<std::string> & requested,
          const std::vector<std::string> & suggested,
          const std::string_view location_id,
          const std::string_view display_name)
        {
          const bool using_suggested_aliases =
            requested.empty();

          const auto & source =
            using_suggested_aliases ?
            suggested :
            requested;

          const auto location_id_key =
            normalize_lookup_key(location_id);

          const auto display_name_key =
            normalize_lookup_key(display_name);

          std::vector<std::string> aliases;
          aliases.reserve(source.size());

          for (const auto & alias : source) {
            const auto cleaned =
              collapse_ascii_whitespace(alias);

            if (using_suggested_aliases) {
              const auto alias_key =
                normalize_lookup_key(cleaned);

              const bool redundant =
                !alias_key.empty() &&
                (
                  alias_key == location_id_key ||
                  alias_key == display_name_key
                );

              if (redundant) {
                continue;
              }
            }

            aliases.push_back(cleaned);
          }

          return aliases;
        }
'''

    source = replace_once(
        source,
        old_function,
        new_function,
        "choose_aliases function",
    )

    old_call = '''          location.aliases =
            choose_aliases(
              request.aliases,
              source.suggested_aliases);
'''

    new_call = '''          location.aliases =
            choose_aliases(
              request.aliases,
              source.suggested_aliases,
              location.location_id,
              location.display_name);
'''

    source = replace_once(
        source,
        old_call,
        new_call,
        "choose_aliases call",
    )

    SOURCE.write_text(
        source,
        encoding="utf-8",
    )

    test = TEST.read_text(encoding="utf-8")

    old_assertion = '''          EXPECT_EQ(
            approval.location
              ->location
              .location_id,
            "A_201");

          EXPECT_EQ(
            approval.location
              ->location
              .map
              .map_id,
            "campus_main");
'''

    new_assertion = '''          EXPECT_EQ(
            approval.location
              ->location
              .location_id,
            "A_201");

          // The inherited suggestion "A 201" becomes
          // redundant after the override canonicalizes
          // the location ID to "A_201".
          EXPECT_TRUE(
            approval.location
              ->location
              .aliases
              .empty());

          EXPECT_EQ(
            approval.location
              ->location
              .map
              .map_id,
            "campus_main");
'''

    test = replace_once(
        test,
        old_assertion,
        new_assertion,
        "redundant inherited alias assertion",
    )

    test_anchor = '''        TEST(LocationCatalog, ApprovalRequiresApproachPose)
'''

    explicit_alias_test = '''        TEST(
          LocationCatalog,
          ExplicitRedundantAliasRemainsInvalid)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          auto request =
            make_approval(
              "candidate-27",
              1U);

          request.location_id = "a 201";

          // Explicit operator input remains strict.
          // It must not be silently removed.
          request.aliases = {
            "A 201",
          };

          const auto result =
            catalog.approve_candidate(request);

          EXPECT_FALSE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::ApprovalCode::
              kInvalidLocation);

          const auto candidate =
            catalog.get_candidate(
              "candidate-27");

          ASSERT_TRUE(candidate.has_value());

          EXPECT_EQ(
            candidate->state,
            savo_locations::CandidateState::
              kPendingReview);

          EXPECT_EQ(
            candidate->candidate_revision,
            1U);
        }


        TEST(LocationCatalog, ApprovalRequiresApproachPose)
'''

    test = replace_once(
        test,
        test_anchor,
        explicit_alias_test,
        "explicit duplicate-alias test insertion",
    )

    TEST.write_text(
        test,
        encoding="utf-8",
    )

    verification = SOURCE.read_text(encoding="utf-8")

    required_source_fragments = (
        "using_suggested_aliases",
        "alias_key == location_id_key",
        "alias_key == display_name_key",
        "location.location_id,",
        "location.display_name);",
    )

    for fragment in required_source_fragments:
        if fragment not in verification:
            raise RuntimeError(
                f"Source verification failed: {fragment}"
            )

    test_verification = TEST.read_text(
        encoding="utf-8"
    )

    required_test_fragments = (
        "ExplicitRedundantAliasRemainsInvalid",
        ".aliases\n              .empty()",
        "CandidateState::\n              kPendingReview",
    )

    for fragment in required_test_fragments:
        if fragment not in test_verification:
            raise RuntimeError(
                f"Test verification failed: {fragment}"
            )

    manifest = (
        LOGS
        / f"LOC1C_alias_fix_{stamp}.sha256"
    )

    manifest.write_text(
        "\n".join(
            (
                f"{sha256(SOURCE)}  "
                "core/savo_locations/"
                "src/location_catalog.cpp"
            ),
            (
                f"{sha256(TEST)}  "
                "core/savo_locations/"
                "test/unit/test_location_catalog.cpp"
            ),
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")

    print(
        "LOC-1C inherited-alias correction applied."
    )

    print(
        "Explicit operator aliases remain strictly validated."
    )


if __name__ == "__main__":
    main()
