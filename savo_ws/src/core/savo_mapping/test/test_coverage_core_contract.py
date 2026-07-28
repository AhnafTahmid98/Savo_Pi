#!/usr/bin/env python3

"""Static ownership and integration contract for the Phase 4L-A1 core."""

from pathlib import Path
import re
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
CORE_FILES = (
    Path('include/savo_mapping/coverage_grid.hpp'),
    Path('src/coverage/coverage_grid.cpp'),
    Path('test/test_coverage_grid.cpp'),
    Path('include/savo_mapping/coverage_planner.hpp'),
    Path('src/coverage/coverage_planner.cpp'),
    Path('test/test_coverage_planner.cpp'),
)
DEFERRED_ASSETS = (
    Path('config/coverage_mapping.yaml'),
    Path('config/profiles/coverage_mapping_real_robot.yaml'),
    Path('launch/coverage_mapping.launch.xml'),
    Path('rviz/coverage_mapping.rviz'),
)


class CoverageCoreContract(unittest.TestCase):

    def test_core_files_are_real_source_files(self) -> None:
        for relative in CORE_FILES:
            path = PACKAGE / relative
            self.assertTrue(path.is_file(), relative)
            self.assertGreater(path.stat().st_size, 0, relative)

    def test_namespace_and_public_semantics_are_locked(self) -> None:
        combined = '\n'.join(
            (PACKAGE / relative).read_text(encoding='utf-8')
            for relative in CORE_FILES
        )
        self.assertIn('namespace savo_mapping::coverage', combined)
        for symbol in (
            'CoverageGridMetadata',
            'CoverageCell',
            'CoverageGridOptions',
            'CoverageGrid',
            'GridIndex',
            'WorldPoint',
            'allow_unknown',
            'inflation_radius_m',
            'reachable_from',
            'CoveragePlannerOptions',
            'CoverageWaypoint',
            'CoveragePlan',
            'CoveragePlanResult',
            'coverage_cells',
            'transit_cells',
            'estimated_coverage_ratio',
            'estimated_path_length_m',
        ):
            self.assertIn(symbol, combined, symbol)

    def test_deferred_deployment_assets_remain_zero_byte(self) -> None:
        for relative in DEFERRED_ASSETS:
            path = PACKAGE / relative
            self.assertTrue(path.is_file(), relative)
            self.assertEqual(path.stat().st_size, 0, relative)

    def test_core_has_no_ros_or_execution_authority(self) -> None:
        forbidden = re.compile(
            r'(?:rclcpp|nav_msgs|geometry_msgs|sensor_msgs|'
            r'NavigateToPose|FollowWaypoints|RotateToHeading|'
            r'create_(?:publisher|subscription|client|service)|'
            r'cmd_vel|Twist|savo_control|savo_nav)'
        )
        for relative in CORE_FILES:
            text = (PACKAGE / relative).read_text(encoding='utf-8')
            self.assertIsNone(forbidden.search(text), relative)

    def test_core_does_not_reference_generated_artifacts(self) -> None:
        forbidden = re.compile(r'(?:^|[/\'])(?:build|install|log)/')
        for relative in CORE_FILES:
            text = (PACKAGE / relative).read_text(encoding='utf-8')
            self.assertIsNone(forbidden.search(text), relative)

    def test_cmake_builds_and_tests_both_core_modules(self) -> None:
        cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
        required = (
            'src/coverage/coverage_grid.cpp',
            'src/coverage/coverage_planner.cpp',
            'include/savo_mapping/coverage_grid.hpp',
            'include/savo_mapping/coverage_planner.hpp',
            'ament_add_gtest(test_coverage_grid',
            'ament_add_gtest(test_coverage_planner',
            'ament_add_pytest_test(\n    test_coverage_core_contract',
        )
        for contract in required:
            self.assertIn(contract, cmake, contract)
        for exactly_once in (
            'src/coverage/coverage_grid.cpp',
            'src/coverage/coverage_planner.cpp',
            'include/savo_mapping/coverage_grid.hpp',
            'include/savo_mapping/coverage_planner.hpp',
            'ament_add_gtest(test_coverage_grid',
            'ament_add_gtest(test_coverage_planner',
            'test_coverage_core_contract.py',
        ):
            self.assertEqual(cmake.count(exactly_once), 1, exactly_once)

    def test_stable_validation_reasons_remain_present(self) -> None:
        combined = '\n'.join(
            (PACKAGE / relative).read_text(encoding='utf-8')
            for relative in CORE_FILES
        )
        reasons = (
            'coverage_grid_dimensions_invalid',
            'coverage_grid_resolution_invalid',
            'coverage_grid_origin_invalid',
            'coverage_grid_data_size_mismatch',
            'coverage_grid_thresholds_invalid',
            'coverage_grid_start_out_of_bounds',
            'coverage_grid_start_blocked',
            'coverage_grid_no_reachable_space',
            'coverage_planner_track_spacing_invalid',
            'coverage_planner_waypoint_limit_invalid',
            'coverage_planner_no_coverage_cells',
            'coverage_planner_connection_failed',
            'coverage_planner_waypoint_limit_exceeded',
            'coverage_plan_ready',
        )
        for reason in reasons:
            self.assertIn(reason, combined, reason)


if __name__ == '__main__':
    unittest.main()
