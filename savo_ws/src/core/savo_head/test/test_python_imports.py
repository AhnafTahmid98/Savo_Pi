import importlib
import py_compile
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


PURE_IMPORT_MODULES = [
    "savo_head",
    "savo_head.version",
    "savo_head.constants",
    "savo_head.contracts",
    "savo_head.contracts.topic_names",
    "savo_head.contracts.frame_names",
    "savo_head.contracts.parameter_names",
    "savo_head.models",
    "savo_head.models.pantilt_command",
    "savo_head.models.scan_status",
    "savo_head.models.semantic_confirmation",
    "savo_head.core",
    "savo_head.core.calibration",
    "savo_head.core.scan_pattern",
    "savo_head.core.validation",
    "savo_head.drivers",
    "savo_head.drivers.pca9685_driver",
    "savo_head.drivers.pantilt_driver",
    "savo_head.tools",
    "savo_head.tools.head_camera_view",
    "savo_head.tools.head_manual_cli",
    "savo_head.tools.apriltag_debug_cli",
    "savo_head.tools.dump_effective_head_params",
]


PYTHON_SOURCE_FILES = [
    "savo_head/nodes/head_controller_node.py",
    "savo_head/nodes/head_scan_node.py",
    "savo_head/nodes/head_tf_node.py",
    "savo_head/nodes/head_status_node.py",
    "savo_head/nodes/apriltag_confirm_node.py",
    "scripts/head_camera_view.py",
    "scripts/head_manual_cli.py",
    "scripts/apriltag_debug_cli.py",
    "scripts/dump_effective_head_params.py",
    "scripts/head_controller_node_py",
    "scripts/head_scan_node_py",
    "scripts/head_tf_node_py",
    "scripts/head_status_node_py",
    "scripts/apriltag_confirm_node_py",
]


def import_module(name: str):
    return importlib.import_module(name)


def require_attr(module, candidates: list[str]) -> None:
    assert any(hasattr(module, item) for item in candidates), (
        f"{module.__name__} missing one of expected attrs: {candidates}"
    )


def test_pure_python_modules_import():
    for module_name in PURE_IMPORT_MODULES:
        import_module(module_name)


def test_python_source_files_compile():
    for relative_path in PYTHON_SOURCE_FILES:
        path = PACKAGE_ROOT / relative_path
        assert path.is_file(), f"Missing Python source file: {relative_path}"
        py_compile.compile(str(path), doraise=True)


def test_nodes_package_is_lazy_import_safe():
    nodes = import_module("savo_head.nodes")

    require_attr(
        nodes,
        [
            "__all__",
            "NODE_MODULES",
            "NODE_ENTRYPOINTS",
            "get_node_entrypoint",
        ],
    )


def test_constants_module_imports_safely():
    constants = import_module("savo_head.constants")

    public_names = [
        name for name in vars(constants)
        if not name.startswith("_")
    ]

    assert public_names, "savo_head.constants should expose at least one public constant"

def test_topic_contract_module_exports_head_topics():
    topics = import_module("savo_head.contracts.topic_names")

    text = "\n".join(
        str(value)
        for value in vars(topics).values()
        if isinstance(value, str)
    )

    for required in [
        "/savo_head/pan_tilt_cmd",
        "/savo_head/pan_tilt_state",
        "/savo_head/status",
        "/savo_head/dashboard_text",
        "/savo_head/semantic_confirmations",
    ]:
        assert required in text, f"Missing topic contract: {required}"


def test_frame_contract_module_exports_head_frames():
    frames = import_module("savo_head.contracts.frame_names")

    text = "\n".join(
        str(value)
        for value in vars(frames).values()
        if isinstance(value, str)
    )

    for required in [
        "base_link",
        "pantilt_pan_link",
        "pantilt_tilt_link",
        "pi_camera_link",
        "pi_camera_optical_frame",
    ]:
        assert required in text, f"Missing frame contract: {required}"


def test_calibration_module_locks_pan_tilt_mapping():
    calibration = import_module("savo_head.core.calibration")

    assert hasattr(calibration, "logical_channel_to_pca9685_channel")
    assert calibration.logical_channel_to_pca9685_channel("7") == 15
    assert calibration.logical_channel_to_pca9685_channel("6") == 14

    if hasattr(calibration, "default_servo_calibration"):
        config = calibration.default_servo_calibration()
        assert config is not None


def test_scan_pattern_module_exports_staged_scan():
    scan_pattern = import_module("savo_head.core.scan_pattern")

    names = vars(scan_pattern)

    assert any("Scan" in name for name in names), "scan_pattern module missing Scan types"
    assert any("profile" in name.lower() for name in names), "scan_pattern module missing profile support"


def test_model_modules_export_domain_types():
    pantilt = import_module("savo_head.models.pantilt_command")
    scan = import_module("savo_head.models.scan_status")
    semantic = import_module("savo_head.models.semantic_confirmation")

    assert any("Command" in name for name in vars(pantilt)), "pantilt_command missing command model"
    assert any("Scan" in name for name in vars(scan)), "scan_status missing scan model"
    assert any("Confirmation" in name for name in vars(semantic)), (
        "semantic_confirmation missing confirmation model"
    )


def test_tools_have_main_or_cli_entrypoints():
    for module_name in [
        "savo_head.tools.head_camera_view",
        "savo_head.tools.head_manual_cli",
        "savo_head.tools.apriltag_debug_cli",
        "savo_head.tools.dump_effective_head_params",
    ]:
        module = import_module(module_name)
        assert any(
            hasattr(module, candidate)
            for candidate in ["main", "run", "build_arg_parser", "parse_args"]
        ), f"{module_name} missing CLI entrypoint helpers"


def test_python_fallback_wrappers_have_shebang_or_imports():
    wrappers = [
        "scripts/head_controller_node_py",
        "scripts/head_scan_node_py",
        "scripts/head_tf_node_py",
        "scripts/head_status_node_py",
        "scripts/apriltag_confirm_node_py",
    ]

    for relative_path in wrappers:
        path = PACKAGE_ROOT / relative_path
        text = path.read_text()
        assert "savo_head" in text or text.startswith("#!"), f"Wrapper looks invalid: {relative_path}"


if __name__ == "__main__":
    test_pure_python_modules_import()
    test_python_source_files_compile()
    test_nodes_package_is_lazy_import_safe()
    test_constants_module_imports_safely()
    test_topic_contract_module_exports_head_topics()
    test_frame_contract_module_exports_head_frames()
    test_calibration_module_locks_pan_tilt_mapping()
    test_scan_pattern_module_exports_staged_scan()
    test_model_modules_export_domain_types()
    test_tools_have_main_or_cli_entrypoints()
    test_python_fallback_wrappers_have_shebang_or_imports()
    print("PASS: savo_head Python modules, tools, and fallback wrappers import/compile safely.")
