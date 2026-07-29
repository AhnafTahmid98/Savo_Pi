from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(
        encoding="utf-8"
    )


def parse_version(
    value: str,
) -> tuple[int, int, int]:
    return tuple(
        int(part)
        for part in value.split(".")
    )


def test_package_is_loc3a_or_later() -> None:
    package = ET.parse(
        ROOT / "package.xml"
    ).getroot()

    version = package.findtext("version")

    assert version is not None
    assert parse_version(version) >= (0, 8, 0)

    dependencies = {
        element.text
        for tag in (
            "depend",
            "build_depend",
            "build_export_depend",
            "exec_depend",
        )
        for element in package.findall(tag)
    }

    for dependency in (
        "rclcpp",
        "std_msgs",
        "geometry_msgs",
        "builtin_interfaces",
        "savo_msgs",
        "launch",
        "launch_ros",
    ):
        assert dependency in dependencies


def test_ros_runtime_files_exist() -> None:
    for relative in (
        "include/savo_locations/"
        "read_only_catalog_view.hpp",
        "include/savo_locations/"
        "ros_conversions.hpp",
        "include/savo_locations/"
        "location_registry_node.hpp",
        "src/read_only_catalog_view.cpp",
        "src/ros_conversions.cpp",
        "src/location_registry_node.cpp",
        "src/location_registry_main.cpp",
        "config/locations_node.yaml",
        "launch/locations_bringup.launch.py",
    ):
        assert (ROOT / relative).is_file(), relative


def test_cmake_builds_production_node() -> None:
    cmake = read("CMakeLists.txt")

    for fragment in (
        "add_library(",
        "savo_locations_ros",
        "add_executable(",
        "savo_locations_node",
        "src/location_registry_node.cpp",
        "src/location_registry_main.cpp",
        "rclcpp",
        "std_msgs",
        "test_registry_node",
        "test_phase3a_contracts",
    ):
        assert fragment in cmake


def test_node_uses_persistent_bootstrap() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    assert "SqliteStore" in source
    assert "SqliteRepository" in source
    assert "migrate(" in source
    assert "bootstrap(" in source
    assert "catalog_view_.replace(" in source

    assert (
        "/var/lib/robot_savo/locations/"
        "locations.db"
        in source
    )

    assert '"degraded"' in source
    assert "registry unavailable:" in source


def test_node_preserves_loc3a_read_services() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    header = read(
        "include/savo_locations/"
        "location_registry_node.hpp"
    )

    combined = source + header

    for endpoint in (
        "service_names::kResolve",
        "service_names::kGet",
        "service_names::kList",
    ):
        assert endpoint in combined

    # LOC-3A introduced a read-only runtime. Later phases may add
    # write services, but must preserve the original read surface.
    package = ET.parse(ROOT / "package.xml").getroot()
    version = package.findtext("version")

    assert version is not None

    if parse_version(version) < (0, 10, 0):
        for forbidden in (
            "RegisterLocationCandidate",
            "ApproveLocation",
            "SetLocationEnabled",
            "handle_register",
            "handle_approve",
            "handle_set_enabled",
        ):
            assert forbidden not in combined



def test_status_snapshot_latched_heartbeat_volatile() -> None:
    source = read(
        "src/location_registry_node.cpp"
    )

    topics = read(
        "include/savo_locations/topic_names.hpp"
    )

    assert "/savo_locations/status" in topics
    assert "/savo_locations/heartbeat" in topics
    assert "/savo_locations/snapshot" in topics

    # Status and snapshot retain their latest values.
    assert "transient_local()" in source

    # Heartbeat is a continuous volatile signal.
    assert "durability_volatile()" in source

    # Status/snapshot identify LOC-3A as read-only.
    assert "mode" in source
    assert "read_only" in source



def test_resolution_remains_fail_closed() -> None:
    view = read(
        "src/read_only_catalog_view.cpp"
    )

    assert "Exact canonical-ID precedence" in view
    assert "location is disabled" in view
    assert "location is retired" in view
    assert "location query is ambiguous" in view
    assert "another map context" in view


def test_launch_uses_installed_configuration() -> None:
    launch = read(
        "launch/locations_bringup.launch.py"
    )

    assert (
        'get_package_share_directory("savo_locations")'
        in launch
    )

    assert 'executable="savo_locations_node"' in launch
    assert 'parameters=[config_file]' in launch


def test_loc3a_tests_are_registered() -> None:
    cmake = read("CMakeLists.txt")

    for target in (
        "test_read_only_catalog_view",
        "test_registry_node",
        "test_phase3a_contracts",
    ):
        assert target in cmake
