import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


VALID_SCAN_PARAMS = {
    "semantic_scan_enabled": True,
    "semantic_scan_mode": "staged",
    "semantic_scan_pan_min_deg": 0,
    "semantic_scan_pan_center_deg": 72,
    "semantic_scan_pan_max_deg": 170,
    "semantic_scan_tilt_min_deg": 45,
    "semantic_scan_tilt_max_deg": 130,
    "semantic_scan_pan_step_deg": 2,
    "semantic_scan_tilt_step_deg": 2,
    "semantic_scan_step_delay_s": 0.05,
    "semantic_scan_start_pan_deg": 0,
    "semantic_scan_start_tilt_deg": 45,
    "semantic_scan_pan_targets_deg": [72, 170, 72, 0],
    "semantic_scan_tilt_sweep_pan_targets_deg": [72],
    "semantic_scan_hold_at_pan_target_s": 0.20,
    "semantic_scan_hold_after_tilt_sweep_s": 0.20,
    "semantic_scan_pause_on_manual_command": True,
    "semantic_scan_resume_after_manual_s": 2.0,
    "semantic_scan_center_on_stop": True,
}


def profile_values(profile):
    return {
        "pan_targets_deg": tuple(getattr(profile, "pan_targets_deg")),
        "tilt_sweep_pan_targets_deg": tuple(getattr(profile, "tilt_sweep_pan_targets_deg")),
        "pan_min_deg": int(getattr(profile, "pan_min_deg")),
        "pan_center_deg": int(getattr(profile, "pan_center_deg")),
        "pan_max_deg": int(getattr(profile, "pan_max_deg")),
        "tilt_min_deg": int(getattr(profile, "tilt_min_deg")),
        "tilt_max_deg": int(getattr(profile, "tilt_max_deg")),
        "pan_step_deg": int(getattr(profile, "pan_step_deg")),
        "tilt_step_deg": int(getattr(profile, "tilt_step_deg")),
        "start_pan_deg": int(getattr(profile, "start_pan_deg")),
        "start_tilt_deg": int(getattr(profile, "start_tilt_deg")),
    }


def test_profile_from_params_locks_staged_scan_pattern():
    from savo_head.core.scan_pattern import profile_from_params

    profile = profile_from_params(dict(VALID_SCAN_PARAMS))
    values = profile_values(profile)

    assert values["pan_targets_deg"] == (72, 170, 72, 0)
    assert values["tilt_sweep_pan_targets_deg"] == (72,)
    assert values["pan_min_deg"] == 0
    assert values["pan_center_deg"] == 72
    assert values["pan_max_deg"] == 170
    assert values["tilt_min_deg"] == 45
    assert values["tilt_max_deg"] == 130
    assert values["pan_step_deg"] == 2
    assert values["tilt_step_deg"] == 2
    assert values["start_pan_deg"] == 0
    assert values["start_tilt_deg"] == 45


def test_valid_scan_profile_has_no_validation_errors():
    from savo_head.core.scan_pattern import profile_from_params

    profile = profile_from_params(dict(VALID_SCAN_PARAMS))

    assert hasattr(profile, "validation_errors"), "Scan profile missing validation_errors()"
    assert not profile.validation_errors(), profile.validation_errors()


def test_scan_profile_normalizes_bad_values_to_safe_defaults():
    from savo_head.core.scan_pattern import profile_from_params

    params = dict(VALID_SCAN_PARAMS)
    params["semantic_scan_pan_targets_deg"] = []
    params["semantic_scan_tilt_max_deg"] = 20

    profile = profile_from_params(params)
    values = profile_values(profile)

    assert values["pan_targets_deg"], "normalized scan profile must keep pan targets"
    assert values["tilt_max_deg"] >= values["tilt_min_deg"]
    assert not profile.validation_errors(), profile.validation_errors()

def test_scan_module_exposes_runtime_or_step_helpers():
    from savo_head.core import scan_pattern

    public_names = {
        name
        for name in vars(scan_pattern)
        if not name.startswith("_")
    }

    assert any("profile" in name.lower() for name in public_names), (
        "scan_pattern module must expose profile helpers"
    )

    assert any(
        token in name.lower()
        for name in public_names
        for token in ("scan", "runtime", "step", "pattern")
    ), "scan_pattern module must expose scan/runtime/step helpers"


if __name__ == "__main__":
    test_profile_from_params_locks_staged_scan_pattern()
    test_valid_scan_profile_has_no_validation_errors()
    test_scan_profile_normalizes_bad_values_to_safe_defaults()
    test_scan_module_exposes_runtime_or_step_helpers()
    print("PASS: savo_head Python scan pattern locks staged pan/tilt behavior.")
