from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def require_file(relative_path: str) -> None:
    path = PACKAGE_ROOT / relative_path
    assert path.is_file(), f"Missing file: {relative_path}"


def require_dir(relative_path: str) -> None:
    path = PACKAGE_ROOT / relative_path
    assert path.is_dir(), f"Missing directory: {relative_path}"


def test_top_level_package_layout():
    for directory in [
        "config",
        "include",
        "launch",
        "resource",
        "scripts",
        "savo_head",
        "src",
        "test",
    ]:
        require_dir(directory)

    for file_path in [
        "CMakeLists.txt",
        "package.xml",
        "setup.cfg",
        "setup.py",
        "resource/savo_head",
    ]:
        require_file(file_path)


def test_cpp_include_layout():
    for file_path in [
        "include/savo_head/core/head_types.hpp",
        "include/savo_head/core/servo_calibration.hpp",
        "include/savo_head/core/scan_pattern.hpp",
        "include/savo_head/core/diagnostics.hpp",
        "include/savo_head/core/topic_contract.hpp",
        "include/savo_head/core/frame_contract.hpp",
        "include/savo_head/core/head_state.hpp",
        "include/savo_head/core/pantilt_command.hpp",
        "include/savo_head/core/head_config.hpp",
        "include/savo_head/core/head_status.hpp",
        "include/savo_head/drivers/pca9685_driver.hpp",
        "include/savo_head/drivers/pantilt_driver.hpp",
    ]:
        require_file(file_path)


def test_cpp_source_layout():
    for file_path in [
        "src/core/head_state.cpp",
        "src/core/scan_pattern.cpp",
        "src/core/servo_calibration.cpp",
        "src/core/head_config.cpp",
        "src/core/head_status.cpp",
        "src/drivers/pca9685_driver.cpp",
        "src/drivers/pantilt_driver.cpp",
        "src/nodes/head_controller_node.cpp",
        "src/nodes/head_scan_node.cpp",
        "src/nodes/head_tf_node.cpp",
        "src/nodes/head_status_node.cpp",
        "src/nodes/apriltag_confirm_node.cpp",
    ]:
        require_file(file_path)


def test_python_package_layout():
    for file_path in [
        "savo_head/__init__.py",
        "savo_head/constants.py",
        "savo_head/version.py",
        "savo_head/contracts/__init__.py",
        "savo_head/contracts/topic_names.py",
        "savo_head/contracts/frame_names.py",
        "savo_head/contracts/parameter_names.py",
        "savo_head/core/__init__.py",
        "savo_head/core/calibration.py",
        "savo_head/core/scan_pattern.py",
        "savo_head/core/validation.py",
        "savo_head/drivers/__init__.py",
        "savo_head/drivers/pca9685_driver.py",
        "savo_head/drivers/pantilt_driver.py",
        "savo_head/models/__init__.py",
        "savo_head/models/pantilt_command.py",
        "savo_head/models/scan_status.py",
        "savo_head/models/semantic_confirmation.py",
        "savo_head/nodes/__init__.py",
        "savo_head/nodes/head_controller_node.py",
        "savo_head/nodes/head_scan_node.py",
        "savo_head/nodes/head_tf_node.py",
        "savo_head/nodes/head_status_node.py",
        "savo_head/nodes/apriltag_confirm_node.py",
        "savo_head/tools/__init__.py",
        "savo_head/tools/head_camera_view.py",
        "savo_head/tools/head_manual_cli.py",
        "savo_head/tools/apriltag_debug_cli.py",
        "savo_head/tools/dump_effective_head_params.py",
    ]:
        require_file(file_path)


def test_script_wrapper_layout():
    for file_path in [
        "scripts/head_camera_view.py",
        "scripts/head_manual_cli.py",
        "scripts/apriltag_debug_cli.py",
        "scripts/dump_effective_head_params.py",
        "scripts/head_controller_node_py",
        "scripts/head_scan_node_py",
        "scripts/head_tf_node_py",
        "scripts/head_status_node_py",
        "scripts/apriltag_confirm_node_py",
    ]:
        require_file(file_path)


def test_launch_layout():
    for file_path in [
        "launch/head_bringup.launch.py",
        "launch/head_camera_stream.launch.py",
        "launch/head_camera_view.launch.py",
        "launch/head_debug.launch.py",
        "launch/head_fallback.launch.py",
        "launch/head_hw_only.launch.py",
        "launch/head_scan_test.launch.py",
        "launch/head_apriltag.launch.py",
    ]:
        require_file(file_path)


def test_config_layout():
    for file_path in [
        "config/head_hardware.yaml",
        "config/scan_profiles.yaml",
        "config/head_topics.yaml",
        "config/head_frames.yaml",
        "config/camera_stream.yaml",
        "config/apriltag_semantics.yaml",
        "config/diagnostics.yaml",
    ]:
        require_file(file_path)


if __name__ == "__main__":
    test_top_level_package_layout()
    test_cpp_include_layout()
    test_cpp_source_layout()
    test_python_package_layout()
    test_script_wrapper_layout()
    test_launch_layout()
    test_config_layout()
    print("PASS: savo_head package layout is complete.")
