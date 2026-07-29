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
        }'''


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

          // Explicit operator aliases remain strict.
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


def find_function_span(
    text: str,
    signature: str,
) -> tuple[int, int]:
    signature_position = text.find(signature)

    if signature_position < 0:
        raise RuntimeError(
            f"Function signature not found: {signature}"
        )

    line_start = (
        text.rfind("\n", 0, signature_position) + 1
    )

    opening_brace = text.find(
        "{",
        signature_position,
    )

    if opening_brace < 0:
        raise RuntimeError(
            "Function opening brace not found."
        )

    depth = 0
    index = opening_brace
    state = "normal"
    quote = ""

    while index < len(text):
        character = text[index]

        next_character = (
            text[index + 1]
            if index + 1 < len(text)
            else ""
        )

        if state == "line_comment":
            if character == "\n":
                state = "normal"

            index += 1
            continue

        if state == "block_comment":
            if (
                character == "*"
                and next_character == "/"
            ):
                state = "normal"
                index += 2
                continue

            index += 1
            continue

        if state == "string":
            if character == "\\":
                index += 2
                continue

            if character == quote:
                state = "normal"
                quote = ""

            index += 1
            continue

        if (
            character == "/"
            and next_character == "/"
        ):
            state = "line_comment"
            index += 2
            continue

        if (
            character == "/"
            and next_character == "*"
        ):
            state = "block_comment"
            index += 2
            continue

        if character in ('"', "'"):
            state = "string"
            quote = character
            index += 1
            continue

        if character == "{":
            depth += 1

        elif character == "}":
            depth -= 1

            if depth == 0:
                return line_start, index + 1

            if depth < 0:
                raise RuntimeError(
                    "Brace depth became negative."
                )

        index += 1

    raise RuntimeError(
        "Function closing brace not found."
    )


def replace_choose_aliases_function(
    source: str,
) -> str:
    if "const bool using_suggested_aliases" in source:
        return source

    start, end = find_function_span(
        source,
        "std::vector<std::string> choose_aliases(",
    )

    existing = source[start:end]

    required_existing_fragments = (
        "requested",
        "suggested",
        "return aliases;",
    )

    for fragment in required_existing_fragments:
        if fragment not in existing:
            raise RuntimeError(
                "Located function does not match "
                f"choose_aliases: {fragment}"
            )

    return (
        source[:start]
        + NEW_FUNCTION
        + source[end:]
    )


def replace_choose_aliases_call(
    source: str,
) -> str:
    already_updated = (
        "source.suggested_aliases,\n"
        "              location.location_id,\n"
        "              location.display_name);"
    )

    if already_updated in source:
        return source

    pattern = re.compile(
        r"""
        location\.aliases
        \s*=\s*
        choose_aliases
        \s*\(
          \s*request\.aliases
          \s*,
          \s*source\.suggested_aliases
          \s*
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
            "Two-argument choose_aliases call "
            "was not found."
        )

    return updated


def add_alias_empty_assertion(
    test: str,
) -> str:
    if (
        "The inherited suggestion"
        in test
    ):
        return test

    test_position = test.find(
        "CandidateOwnsMapAndTagOnApproval"
    )

    if test_position < 0:
        raise RuntimeError(
            "CandidateOwnsMapAndTagOnApproval "
            "test was not found."
        )

    assertion_end = test.find(
        '"A_201");',
        test_position,
    )

    if assertion_end < 0:
        raise RuntimeError(
            "A_201 assertion was not found."
        )

    assertion_end += len('"A_201");')

    addition = '''

          // The inherited suggestion "A 201" becomes
          // redundant when the final ID becomes A_201.
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


def add_explicit_alias_test(
    test: str,
) -> str:
    if (
        "ExplicitRedundantAliasRemainsInvalid"
        in test
    ):
        return test

    anchor_pattern = re.compile(
        r"""
        ^[ \t]*TEST
        \s*\(
        \s*LocationCatalog
        \s*,
        \s*ApprovalRequiresApproachPose
        \s*\)
        """,
        re.MULTILINE | re.VERBOSE,
    )

    match = anchor_pattern.search(test)

    if match is None:
        raise RuntimeError(
            "ApprovalRequiresApproachPose "
            "test insertion point was not found."
        )

    line_start = (
        test.rfind("\n", 0, match.start()) + 1
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
        / f"pre_LOC1C_alias_brace_fix_{stamp}.tar.gz"
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

    source = SOURCE.read_text(
        encoding="utf-8"
    )

    test = TEST.read_text(
        encoding="utf-8"
    )

    source = replace_choose_aliases_function(
        source
    )

    source = replace_choose_aliases_call(
        source
    )

    test = add_alias_empty_assertion(
        test
    )

    test = add_explicit_alias_test(
        test
    )

    required_source_fragments = (
        "const bool using_suggested_aliases",
        "alias_key == location_id_key",
        "alias_key == display_name_key",
        "source.suggested_aliases,",
        "location.location_id,",
        "location.display_name);",
    )

    for fragment in required_source_fragments:
        if fragment not in source:
            raise RuntimeError(
                "Source verification failed: "
                + fragment
            )

    required_test_fragments = (
        "CandidateOwnsMapAndTagOnApproval",
        "ExplicitRedundantAliasRemainsInvalid",
        "The inherited suggestion",
    )

    for fragment in required_test_fragments:
        if fragment not in test:
            raise RuntimeError(
                "Test verification failed: "
                + fragment
            )

    # Files are written only after all transformations
    # and validation complete successfully.
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
        / f"LOC1C_alias_brace_fix_{stamp}.sha256"
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

    print(
        "LOC-1C brace-balanced alias fix applied."
    )

    print(
        "Inherited redundant aliases are removed; "
        "explicit aliases remain strictly validated."
    )


if __name__ == "__main__":
    main()
