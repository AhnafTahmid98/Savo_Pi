#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import hashlib
import re
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


NEW_FUNCTION = '''        std::vector<std::string> choose_aliases(
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


NEW_CALL = '''location.aliases =
            choose_aliases(
              request.aliases,
              source.suggested_aliases,
              location.location_id,
              location.display_name);'''


EXPLICIT_ALIAS_TEST = '''        TEST(
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

          // Explicit operator input remains strict and
          // must not be removed automatically.
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


'''


def sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    return digest.hexdigest()


def replace_choose_aliases_function(source: str) -> str:
    if "const bool using_suggested_aliases" in source:
        return source

    signature = "std::vector<std::string> choose_aliases("

    signature_position = source.find(signature)

    if signature_position < 0:
        raise RuntimeError(
            "Could not find choose_aliases function signature."
        )

    function_start = (
        source.rfind("\n", 0, signature_position) + 1
    )

    # choose_aliases is the final helper inside the
    # anonymous namespace.
    namespace_end = source.find(
        "\n        }  // namespace",
        signature_position,
    )

    if namespace_end < 0:
        raise RuntimeError(
            "Could not find anonymous namespace ending."
        )

    existing_function = source[
        function_start:namespace_end
    ]

    if "return aliases;" not in existing_function:
        raise RuntimeError(
            "Located block does not look like choose_aliases."
        )

    return (
        source[:function_start]
        + NEW_FUNCTION
        + source[namespace_end:]
    )


def replace_choose_aliases_call(source: str) -> str:
    if (
        "source.suggested_aliases,\n"
        "              location.location_id,\n"
        "              location.display_name);"
    ) in source:
        return source

    pattern = re.compile(
        r"""
        location\.aliases
        \s*=\s*
        choose_aliases
        \(
          \s*request\.aliases\s*,
          \s*source\.suggested_aliases\s*
        \)
        \s*;
        """,
        re.VERBOSE,
    )

    updated, count = pattern.subn(
        NEW_CALL,
        source,
        count=1,
    )

    if count != 1:
        raise RuntimeError(
            "Could not replace the two-argument "
            "choose_aliases call."
        )

    return updated


def add_inherited_alias_assertion(test: str) -> str:
    marker = (
        "approval.location\n"
        "              ->location\n"
        "              .aliases\n"
        "              .empty()"
    )

    if marker in test:
        return test

    test_name = "CandidateOwnsMapAndTagOnApproval"
    test_position = test.find(test_name)

    if test_position < 0:
        raise RuntimeError(
            f"Could not find test: {test_name}"
        )

    assertion_end = test.find(
        '"A_201");',
        test_position,
    )

    if assertion_end < 0:
        raise RuntimeError(
            "Could not find A_201 assertion."
        )

    assertion_end += len('"A_201");')

    addition = '''

          // The inherited suggestion "A 201" becomes
          // redundant after the final ID becomes A_201.
          EXPECT_TRUE(
            approval.location
              ->location
              .aliases
              .empty());'''

    return (
        test[:assertion_end]
        + addition
        + test[assertion_end:]
    )


def add_explicit_alias_test(test: str) -> str:
    if "ExplicitRedundantAliasRemainsInvalid" in test:
        return test

    anchor = (
        "TEST(LocationCatalog, "
        "ApprovalRequiresApproachPose)"
    )

    anchor_position = test.find(anchor)

    if anchor_position < 0:
        raise RuntimeError(
            "Could not find ApprovalRequiresApproachPose "
            "test insertion point."
        )

    line_start = (
        test.rfind("\n", 0, anchor_position) + 1
    )

    return (
        test[:line_start]
        + EXPLICIT_ALIAS_TEST
        + test[line_start:]
    )


def main() -> None:
    for path in (SOURCE, TEST):
        if not path.is_file():
            raise SystemExit(
                f"Required file missing: {path}"
            )

    BACKUPS.mkdir(parents=True, exist_ok=True)
    LOGS.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = (
        BACKUPS
        / f"pre_LOC1C_alias_policy_fix_{stamp}.tar.gz"
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
    test = TEST.read_text(encoding="utf-8")

    source = replace_choose_aliases_function(source)
    source = replace_choose_aliases_call(source)

    test = add_inherited_alias_assertion(test)
    test = add_explicit_alias_test(test)

    required_source = (
        "const bool using_suggested_aliases",
        "alias_key == location_id_key",
        "alias_key == display_name_key",
        "location.location_id,",
        "location.display_name);",
    )

    for fragment in required_source:
        if fragment not in source:
            raise RuntimeError(
                f"Source verification failed: {fragment}"
            )

    required_test = (
        "CandidateOwnsMapAndTagOnApproval",
        "ExplicitRedundantAliasRemainsInvalid",
        ".aliases\n              .empty()",
    )

    for fragment in required_test:
        if fragment not in test:
            raise RuntimeError(
                f"Test verification failed: {fragment}"
            )

    # Write only after every transformation and
    # verification succeeds.
    SOURCE.write_text(
        source,
        encoding="utf-8",
    )

    TEST.write_text(
        test,
        encoding="utf-8",
    )

    manifest = (
        LOGS
        / f"LOC1C_alias_policy_fix_{stamp}.sha256"
    )

    manifest.write_text(
        (
            f"{sha256(SOURCE)}  "
            "core/savo_locations/"
            "src/location_catalog.cpp\n"
        )
        + (
            f"{sha256(TEST)}  "
            "core/savo_locations/"
            "test/unit/test_location_catalog.cpp\n"
        ),
        encoding="utf-8",
    )

    print(f"Permanent backup : {backup}")
    print(f"Permanent manifest: {manifest}")
    print("LOC-1C alias-policy correction applied.")
    print(
        "Inherited redundant aliases are removed; "
        "explicit aliases remain strict."
    )


if __name__ == "__main__":
    main()
