import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_identity_and_spatial_confirmation_policy() -> None:
    """Compile the production evidence-mode contract without ROS dependencies."""
    with tempfile.TemporaryDirectory(prefix="savo_head_apriltag_policy_") as temp:
        temp_path = Path(temp)
        source = temp_path / "test.cpp"
        executable = temp_path / "test"
        source.write_text(
            r'''
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "savo_head/core/apriltag_action_contract.hpp"

int main()
{
  using savo_head::apriltag_contract::Duty;
  using savo_head::apriltag_contract::ClassifyIdentityEvidence;
  using savo_head::apriltag_contract::HasMinimumEvidence;
  using savo_head::apriltag_contract::IdentityEvidenceDisposition;
  using savo_head::apriltag_contract::IsIdentityOnlyArrival;
  using savo_head::apriltag_contract::RequiresSpatialEvidence;

  constexpr auto register_location =
    static_cast<std::uint8_t>(Duty::kRegisterLocation);
  constexpr auto confirm_arrival =
    static_cast<std::uint8_t>(Duty::kConfirmArrival);

  // Identity-only arrival must not require camera pose, TF, or spatial evidence.
  static_assert(IsIdentityOnlyArrival(confirm_arrival, false));
  static_assert(!RequiresSpatialEvidence(confirm_arrival, false));

  // A map-pose-required arrival retains the full spatial contract.
  static_assert(!IsIdentityOnlyArrival(confirm_arrival, true));
  static_assert(RequiresSpatialEvidence(confirm_arrival, true));

  // Registration is always spatial, even if a caller clears require_map_pose.
  static_assert(!IsIdentityOnlyArrival(register_location, false));
  static_assert(RequiresSpatialEvidence(register_location, false));
  static_assert(RequiresSpatialEvidence(register_location, true));

  // Camera pose is deliberately absent from identity evidence classification.
  static_assert(
    ClassifyIdentityEvidence(true, true, true, true, true) ==
    IdentityEvidenceDisposition::kAccepted);
  static_assert(
    ClassifyIdentityEvidence(true, false, true, true, true) ==
    IdentityEvidenceDisposition::kWrongTag);
  static_assert(
    ClassifyIdentityEvidence(true, true, false, true, true) ==
    IdentityEvidenceDisposition::kUnstable);
  static_assert(
    ClassifyIdentityEvidence(true, true, true, false, true) ==
    IdentityEvidenceDisposition::kUnstable);
  static_assert(
    ClassifyIdentityEvidence(true, true, true, true, false) ==
    IdentityEvidenceDisposition::kUnstable);

  constexpr std::size_t minimum_observations = 5U;
  std::size_t accepted_observations = 0U;
  for (std::size_t index = 0U; index < minimum_observations; ++index) {
    if (ClassifyIdentityEvidence(true, true, true, true, true) ==
      IdentityEvidenceDisposition::kAccepted)
    {
      ++accepted_observations;
    }
  }
  assert(HasMinimumEvidence(accepted_observations, minimum_observations));
  return 0;
}
''',
            encoding="utf-8",
        )

        compile_result = subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Wpedantic",
                "-I",
                str(ROOT / "include"),
                str(source),
                "-o",
                str(executable),
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        assert compile_result.returncode == 0, (
            compile_result.stdout + compile_result.stderr
        )

        run_result = subprocess.run(
            [str(executable)],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        assert run_result.returncode == 0, run_result.stdout + run_result.stderr
