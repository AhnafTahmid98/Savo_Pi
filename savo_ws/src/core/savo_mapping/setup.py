from setuptools import find_packages, setup

package_name = "savo_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahnaf Tahmid",
    maintainer_email="tahmidahnaf998@gmail.com",
    description=(
        "Robot Savo mapping package with manual mapping, autonomous mapping, "
        "SLAM toolbox integration, map tools, diagnostics, and future semantic "
        "landmark support."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapping_supervisor_node_py = savo_mapping.nodes.mapping_supervisor_node:main",
            "mapping_mode_manager_node_py = savo_mapping.nodes.mapping_mode_manager_node:main",
            "pointcloud_monitor_node_py = savo_mapping.nodes.pointcloud_monitor_node:main",
            "mapping_dashboard_node_py = savo_mapping.nodes.mapping_dashboard_node:main",
            "frontier_explorer_node = savo_mapping.nodes.frontier_explorer_node:main",
            "location_bridge_node = savo_mapping.nodes.location_bridge_node:main",
            "location_confirmation_node = savo_mapping.nodes.location_confirmation_node:main",
            "apriltag_mapper_node = savo_mapping.nodes.apriltag_mapper_node:main",
            "mapping_readiness_cli = scripts.mapping_readiness_cli:main",
            "mapping_smoke_test_cli = scripts.mapping_smoke_test_cli:main",
            "dump_effective_params = scripts.dump_effective_params:main",
            "map_quality_cli = scripts.map_quality_cli:main",
            "map_metadata_cli = scripts.map_metadata_cli:main",
            "load_map_check_cli = scripts.load_map_check_cli:main",
            "save_map_cli = scripts.save_map_cli:main",
            "manual_mapping_cli = scripts.manual_mapping_cli:main",
            "autonomous_mapping_cli = scripts.autonomous_mapping_cli:main",
            "pointcloud_echo_cli = scripts.pointcloud_echo_cli:main",
            "voxel_costmap_check_cli = scripts.voxel_costmap_check_cli:main",
            "apriltag_check_cli = scripts.apriltag_check_cli:main",
            "semantic_landmark_cli = scripts.semantic_landmark_cli:main",
            "location_bridge_check_cli = scripts.location_bridge_check_cli:main",
            "clean_maps_cli = scripts.clean_maps_cli:main",
            "operator_notes_cli = scripts.operator_notes_cli:main",
        ],
    },
)
