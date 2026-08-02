# Pre-real-test zero-byte inventory

This is the immutable RT-0 baseline captured before implementation. It scans
authoritative `deploy`, `docs`, `savo_ws/src`, and `tools` trees. Generated
`build`, `install`, `log`, `.git`, cache, attachment, and tool-owned worktree
copies are intentionally excluded.

Total: **222**

| Category | Count |
| --- | ---: |
| KEEP_INTENTIONALLY_EMPTY | 22 |
| IMPLEMENT | 36 |
| REPLACE | 5 |
| REMOVE_AFTER_REFERENCE_PROOF | 125 |
| DEFER_WITH_REASON | 34 |

| Path | Package | Type | Category | Installed | Launched | Reason |
| --- | --- | --- | --- | --- | --- | --- |
| `deploy/network/chrony_core.conf` | deploy | conf | IMPLEMENT | False | False | required deployment/runtime configuration |
| `deploy/network/chrony_edge.conf` | deploy | conf | IMPLEMENT | False | False | required deployment/runtime configuration |
| `deploy/network/core_netplan.yaml` | deploy | yaml | IMPLEMENT | False | False | required deployment/runtime configuration |
| `deploy/network/edge_netplan.yaml` | deploy | yaml | IMPLEMENT | False | False | required deployment/runtime configuration |
| `deploy/systemd/savo.service` | deploy | service | IMPLEMENT | False | False | required deployment/runtime configuration |
| `deploy/systemd/savo_mapping.service` | savo_mapping.service | service | IMPLEMENT | False | False | required deployment/runtime configuration |
| `docs/architecture/network_architecture.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/architecture/savo_core_architecture.md` | savo_core_architecture.md | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/architecture/speech_intent_flow.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/diagrams/robot_network.drawio` | docs | drawio | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/diagrams/ros_graph.drawio` | docs | drawio | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/diagrams/speech_intent_flow.drawio` | docs | drawio | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/diagrams/two_pi_compute_split.drawio` | docs | drawio | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/packages/savo_vo.md` | savo_vo.md | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/setup/audio_setup.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/setup/ethernet_core_edge_setup.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/setup/realsense_setup.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/setup/tailscale_setup.md` | docs | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `docs/setup/time_sync.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/base_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/full_robot_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/localization_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/perception_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/speech_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `docs/testing/vo_test_plan.md` | docs | md | IMPLEMENT | False | False | required pre-real-test operator documentation |
| `savo_ws/src/core/savo_base/README.md` | savo_base | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/core/savo_base/resource/savo_base` | savo_base | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_control/README.md` | savo_control | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/core/savo_control/resource/savo_control` | savo_control | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_head/resource/savo_head` | savo_head | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_lidar/resource/savo_lidar` | savo_lidar | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_localization/README.md` | savo_localization | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/core/savo_localization/resource/savo_localization` | savo_localization | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_localization/savo_localization/nodes/ekf_state_publisher_node.py` | savo_localization | py | REPLACE | True | True | empty compatibility implementation replaced by C++ health path and observer visualization |
| `savo_ws/src/core/savo_localization/savo_localization/nodes/localization_dashboard.py` | savo_localization | py | REPLACE | True | True | empty compatibility implementation replaced by C++ health path and observer visualization |
| `savo_ws/src/core/savo_localization/savo_localization/nodes/wheel_odom_fallback_node.py` | savo_localization | py | REPLACE | True | True | empty compatibility implementation replaced by C++ health path and observer visualization |
| `savo_ws/src/core/savo_localization/savo_localization/sensors_api/encoders_api.py` | savo_localization | py | REPLACE | True | False | empty compatibility implementation replaced by C++ health path and observer visualization |
| `savo_ws/src/core/savo_localization/savo_localization/sensors_api/imu_api.py` | savo_localization | py | REPLACE | True | False | empty compatibility implementation replaced by C++ health path and observer visualization |
| `savo_ws/src/core/savo_locations/resource/savo_locations` | savo_locations | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_mapping/config/autonomous_mapping.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/manual_mapping.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/profiles/autonomous_mapping_real_robot.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/profiles/frontier_mapping_real_robot.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/profiles/lab_test.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/profiles/manual_mapping_real_robot.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/realsense_mapping.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/config/voxel_obstacle_monitor.yaml` | savo_mapping | yaml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/map_io.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/map_metrics.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/map_quality.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/map_saver.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/map_validator.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/nav_goal_client.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/realsense_mapping_monitor.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/slam_toolbox_client.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/include/savo_mapping/voxel_obstacle_monitor.hpp` | savo_mapping | hpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/launch/mapping_bringup.launch.xml` | savo_mapping | xml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/launch/realsense_mapping_monitor.launch.xml` | savo_mapping | xml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/launch/slam_toolbox_mapping.launch.xml` | savo_mapping | xml | REMOVE_AFTER_REFERENCE_PROOF | True | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/resource/savo_mapping` | savo_mapping | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/core/savo_mapping/src/nodes/map_quality_node.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/nodes/realsense_mapping_monitor_node.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/nodes/voxel_obstacle_monitor_node.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/perception/realsense_mapping_monitor.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/perception/voxel_obstacle_monitor.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/quality/map_metrics.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/quality/map_quality.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/quality/map_validator.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/ros/nav_goal_client.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/session/map_io.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/session/map_saver.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/session/session_report.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/src/slam/slam_toolbox_client.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/test/test_map_metrics.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/test/test_map_quality.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/test/test_map_validator.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/test/test_realsense_mapping_monitor.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/core/savo_mapping/test/test_voxel_obstacle_monitor.cpp` | savo_mapping | cpp | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy zero-byte mapping path outside active runtime targets |
| `savo_ws/src/edge/savo_realsense/resource/savo_realsense` | savo_realsense | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/edge/savo_ui/include/savo_ui/app/ui_app.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/app/ui_config.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/app/ui_runtime.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/app/ui_state.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/platform/linux_input_reader.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/platform/system_clock.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/platform/touch_input.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/render/animation.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/render/primitive_draw.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/render/rect.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/ros/ros_topics.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/ros/ui_publishers.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/ros/ui_ros_bridge.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/ros/ui_subscribers.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/diagnostics_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/home_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/intro_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/map_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/navigation_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/power_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/safety_overlay.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/screen_base.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/status_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/screens/voice_screen.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/apriltag_confirmation_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/battery_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/bottom_nav.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/button_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/camera_preview_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/network_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/ring_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/safety_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/status_card.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/top_bar.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/include/savo_ui/widgets/wave_widget.hpp` | savo_ui | hpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/resource/savo_ui` | savo_ui | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/edge/savo_ui/scripts/__init__.py` | savo_ui | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `savo_ws/src/edge/savo_ui/scripts/install_savo_ui_service.sh` | savo_ui | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/edge/savo_ui/scripts/uninstall_savo_ui_service.sh` | savo_ui | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/edge/savo_ui/src/app/ui_app.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/app/ui_config.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/app/ui_runtime.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/app/ui_state.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/platform/linux_input_reader.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/platform/system_clock.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/platform/touch_input.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/render/animation.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/render/color.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/render/primitive_draw.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/render/rect.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/ros/ros_topics.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/ros/ui_publishers.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/ros/ui_ros_bridge.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/ros/ui_subscribers.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/diagnostics_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/home_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/intro_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/map_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/navigation_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/power_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/safety_overlay.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/screen_base.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/status_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/screens/voice_screen.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/apriltag_confirmation_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/battery_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/bottom_nav.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/button_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/camera_preview_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/network_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/ring_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/safety_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/status_card.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/top_bar.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/src/widgets/wave_widget.cpp` | savo_ui | cpp | REMOVE_AFTER_REFERENCE_PROOF | True | False | inactive zero-byte modular UI scaffold; active monolithic C++ UI is separate |
| `savo_ws/src/edge/savo_ui/systemd/savo-ui.service` | savo_ui | service | IMPLEMENT | False | False | required deployment/runtime configuration |
| `savo_ws/src/edge/savo_ui/test/test_layout.cpp` | savo_ui | cpp | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/edge/savo_ui/test/test_touch_zones.cpp` | savo_ui | cpp | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/edge/savo_ui/test/test_ui_state.cpp` | savo_ui | cpp | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/edge/savo_vo/resource/savo_vo` | savo_vo | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/shared/savo_bringup/config/nav2_params.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/config/robot_state_publisher.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/config/rplidar.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/config/slam_toolbox.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/params/ekf_odom.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/params/heading.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/params/odom.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/params/sensors.yaml` | savo_bringup | yaml | REMOVE_AFTER_REFERENCE_PROOF | False | False | legacy empty overlay superseded by package-owned configuration |
| `savo_ws/src/shared/savo_bringup/resource/savo_bringup` | savo_bringup | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/shared/savo_description/meshes/README.md` | savo_description | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_description/meshes/chassis/README.md` | savo_description | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_description/meshes/compute/README.md` | savo_description | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_description/meshes/sensors/README.md` | savo_description | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_description/meshes/wheels/README.md` | savo_description | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_perception/resource/savo_perception` | savo_perception | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/shared/savo_perception/savo_perception/__init__.py` | savo_perception | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `savo_ws/src/shared/savo_perception/test/test_imports.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_launch_files.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_param_contract.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_param_loader.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_range_fusion.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_safety_policy.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_topic_contract.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_perception/test/test_topic_names.py` | savo_perception | py | REMOVE_AFTER_REFERENCE_PROOF | False | False | obsolete empty test superseded by active unit/contract tests |
| `savo_ws/src/shared/savo_power/README.md` | savo_power | md | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `savo_ws/src/shared/savo_power/resource/savo_power` | savo_power | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `savo_ws/src/shared/savo_supervisor/resource/savo_supervisor` | savo_supervisor | marker | KEEP_INTENTIONALLY_EMPTY | False | False | ament/resource marker or directory marker |
| `tools/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/dev/check_network.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/dev/check_ros_graph.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/dev/startup_sequence.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/diag/infra/cpu_temp_check.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/infra/diag_utils.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/infra/i2c_scan.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/infra/power_check.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/infra/run_all.sh` | tools | sh | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/infra/tf_tree_check.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/infra/usb_devices.sh` | tools | sh | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/llm/llm_server_test.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/motion/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/diag/motion/odom_calibration.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/motion/odom_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/power/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/diag/power/current_draw_logger.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/power/power_battery.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/power/system_health.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/safety/cmd_vel_gate_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/safety/estop_retry_test.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/safety/estop_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/safety/safety_stop_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/sensors/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/diag/sensors/api/__init__.py` | tools | py | KEEP_INTENTIONALLY_EMPTY | False | False | namespace-only Python package marker |
| `tools/diag/sensors/api/encoders_api.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/sensors/api/imu_api.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/sensors/apriltag_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/status_mode/status_prompt_test.py` | tools | py | DEFER_WITH_REASON | False | False | non-runtime placeholder not required for pre-real-test closure |
| `tools/diag/ui/head_pan_tilt_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/ui/screen_ui_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/voice/asr_topic_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/voice/audio_mic_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/voice/audio_speaker_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
| `tools/diag/voice/tts_topic_test.py` | tools | py | IMPLEMENT | False | False | required safe diagnostic implementation |
